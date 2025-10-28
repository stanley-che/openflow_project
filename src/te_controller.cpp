// te_controller.cpp
// Minimal OpenFlow 1.0 controller wrapped as class OFController (C++17)

#include "of_controller.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <map>
#include <unordered_map>
#include <cerrno>
#include <sstream>
#include <algorithm>
using namespace std;

// Minimal definitions so the linker can find them.
OFController::OFController() : impl_(nullptr) {}
OFController::~OFController() {}

// -----------------------------
// OpenFlow 1.0 wire structures
// -----------------------------
#pragma pack(push, 1)
struct ofp_header { uint8_t version, type; uint16_t length; uint32_t xid; };

enum {
  OFP_VERSION = 0x01,
  OFPT_HELLO=0, OFPT_ERROR, OFPT_ECHO_REQUEST, OFPT_ECHO_REPLY, OFPT_VENDOR,
  OFPT_FEATURES_REQUEST, OFPT_FEATURES_REPLY, OFPT_GET_CONFIG_REQUEST,
  OFPT_GET_CONFIG_REPLY, OFPT_SET_CONFIG, OFPT_PACKET_IN, OFPT_FLOW_REMOVED,
  OFPT_PORT_STATUS, OFPT_PACKET_OUT, OFPT_FLOW_MOD, OFPT_PORT_MOD,
  OFPT_STATS_REQUEST=16, OFPT_STATS_REPLY=17, OFPT_BARRIER_REQUEST=18, OFPT_BARRIER_REPLY=19
};

struct ofp_switch_features {
  ofp_header header;
  uint64_t datapath_id;
  uint32_t n_buffers;
  uint8_t  n_tables; uint8_t pad[3];
  uint32_t capabilities;
  uint32_t actions;
};

struct ofp_switch_config {
  ofp_header header;
  uint16_t flags;
  uint16_t miss_send_len;
};

struct ofp_match {
  uint32_t wildcards;
  uint16_t in_port;
  uint8_t  dl_src[6], dl_dst[6];
  uint16_t dl_vlan;
  uint8_t  dl_vlan_pcp; uint8_t pad1;
  uint16_t dl_type;
  uint8_t  nw_tos, nw_proto;
  uint8_t  nw_src[4], nw_dst[4];
  uint16_t tp_src, tp_dst;
};

enum {
  OFPFW_IN_PORT     = 1<<0,
  OFPFW_DL_VLAN     = 1<<1,
  OFPFW_DL_SRC      = 1<<2,
  OFPFW_DL_DST      = 1<<3,
  OFPFW_DL_TYPE     = 1<<4,
  OFPFW_NW_TOS      = 1<<5,
  OFPFW_NW_PROTO    = 1<<6,
  OFPFW_TP_SRC      = 1<<7,
  OFPFW_TP_DST      = 1<<8,
  OFPFW_DL_VLAN_PCP = 1<<20,
};

struct ofp_action_output { uint16_t type, len; uint16_t port, max_len; };
enum { OFPAT_OUTPUT = 0 };

struct ofp_flow_mod {
  ofp_header header;
  ofp_match match;
  uint64_t cookie;
  uint16_t command;
  uint16_t idle_timeout, hard_timeout;
  uint16_t priority;
  uint32_t buffer_id;
  uint16_t out_port;
  uint16_t flags;
};

enum {
  OFPFC_ADD=0, OFPFC_MODIFY=1, OFPFC_MODIFY_STRICT=2,
  OFPFC_DELETE=3, OFPFC_DELETE_STRICT=4
};

struct ofp_packet_out {
  ofp_header header;
  uint32_t buffer_id;
  uint16_t in_port;
  uint16_t actions_len;
  // actions...
  // payload...
};

struct ofp_packet_in {
  ofp_header header;
  uint32_t buffer_id;
  uint16_t total_len;
  uint16_t in_port;
  uint8_t  reason; uint8_t pad;
  uint8_t  data[0];
};

struct ofp_stats_request {
  ofp_header header;
  uint16_t type;
  uint16_t flags;
  uint8_t  body[0];
};

struct ofp_stats_reply {
  ofp_header header;
  uint16_t type;
  uint16_t flags;
  uint8_t  body[0];
};

enum { OFPST_DESC=0, OFPST_FLOW=1, OFPST_AGGREGATE=2, OFPST_TABLE=3, OFPST_PORT=4 };

struct ofp_port_stats_request { uint16_t port_no; uint8_t pad[6]; };

struct ofp_port_stats {
  uint16_t port_no; uint8_t pad[6];
  uint64_t rx_packets, tx_packets, rx_bytes, tx_bytes;
  uint64_t rx_dropped, tx_dropped, rx_errors, tx_errors;
  uint64_t rx_frame_err, rx_over_err, rx_crc_err, collisions;
};

struct ofp_port_mod {
  ofp_header header;
  uint16_t port_no;
  uint8_t  hw_addr[6];
  uint32_t config, mask;
  uint32_t advertise;
  uint8_t  pad[4];
};

enum { OFPPC_PORT_DOWN = 1<<0 };
enum { OFPP_NONE = 0xffff, OFPP_CONTROLLER = 0xfffd, OFPP_FLOOD = 0xfffb };
enum {
  OFPPF_10MB_HD  = 1<<0,  OFPPF_10MB_FD  = 1<<1,
  OFPPF_100MB_HD = 1<<2,  OFPPF_100MB_FD = 1<<3,
  OFPPF_1GB_HD   = 1<<4,  OFPPF_1GB_FD   = 1<<5,
  OFPPF_10GB_FD  = 1<<6,
};
#pragma pack(pop)

// -----------------------------
// Helpers
// -----------------------------
static uint16_t htobe16_u(uint16_t v){return htons(v);}
static uint32_t htobe32_u(uint32_t v){return htonl(v);}
static uint64_t htobe64_u(uint64_t v){
  uint32_t hi = htonl(uint32_t(v>>32)), lo = htonl(uint32_t(v&0xffffffff));
  return (uint64_t(lo) << 32) | hi;
}
static uint32_t advertise_mask_for_speed(int speedMbps){
  if(speedMbps>=10000) return OFPPF_10GB_FD;
  if(speedMbps>=1000)  return OFPPF_1GB_FD;
  if(speedMbps>=100)   return OFPPF_100MB_FD;
  if(speedMbps>=10)    return OFPPF_10MB_FD;
  return 0;
}
static string mac_to_key(const uint8_t m[6]){
  char buf[32]; snprintf(buf,sizeof(buf),"%02x:%02x:%02x:%02x:%02x:%02x",m[0],m[1],m[2],m[3],m[4],m[5]);
  return string(buf);
}

// =============================
// class OFController (impl.)
// =============================
class OFControllerImpl {
public:
  std::atomic<uint32_t> xid{1};
  std::mutex mtx;
  std::thread loop_thread;
  int listen_fd{-1};
  bool running{false};

  struct SwCtx {
    int fd{-1};
    uint64_t dpid{0};
    std::map<int/*port*/, ofp_port_stats> last_ps;
    // 簡易 L2 學習表：mac -> port
    std::unordered_map<std::string,int> mac2port;
  };
  std::map<int,int> sw_index_to_fd; // sw index (1..N) -> fd
  std::map<int, SwCtx> sw;          // fd -> ctx

  // --- send helpers ---
  static void send_all(int fd, const void* buf, size_t len){
    const uint8_t* p=(const uint8_t*)buf; size_t off=0;
    while(off<len){
      ssize_t w=send(fd, p+off, len-off, 0);
      if(w<=0) throw std::runtime_error("send failed");
      off+=size_t(w);
    }
  }
  static int listen_on(uint16_t port){
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    int on=1; setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_ANY); a.sin_port=htons(port);
    if(bind(fd,(sockaddr*)&a,sizeof(a))<0){ perror("bind"); return -1; }
    if(listen(fd,64)<0){ perror("listen"); close(fd); return -1; }
    return fd;
  }

  void send_hello(int fd){
    ofp_header h{}; h.version=OFP_VERSION; h.type=OFPT_HELLO;
    h.length=htobe16_u(sizeof(h)); h.xid=htobe32_u(xid++);
    send_all(fd,&h,sizeof(h));
  }
  void send_features_req(int fd){
    ofp_header h{}; h.version=OFP_VERSION; h.type=OFPT_FEATURES_REQUEST;
    h.length=htobe16_u(sizeof(h)); h.xid=htobe32_u(xid++);
    send_all(fd,&h,sizeof(h));
  }
  void send_get_config_req(int fd){
    ofp_header h{}; h.version=OFP_VERSION; h.type=OFPT_GET_CONFIG_REQUEST;
    h.length=htobe16_u(sizeof(h)); h.xid=htobe32_u(xid++);
    send_all(fd,&h,sizeof(h));
  }
  void send_set_config(int fd, uint16_t flags/*=0*/, uint16_t miss/*=0xffff*/){
    ofp_switch_config c{}; c.header.version=OFP_VERSION; c.header.type=OFPT_SET_CONFIG;
    c.flags=htobe16_u(flags); c.miss_send_len=htobe16_u(miss);
    c.header.length=htobe16_u(sizeof(c)); c.header.xid=htobe32_u(xid++);
    send_all(fd,&c,sizeof(c));
  }
  void send_echo_reply(int fd, const ofp_header* req, const uint8_t* payload, size_t len){
    std::vector<uint8_t> buf(sizeof(ofp_header)+len);
    auto* h=(ofp_header*)buf.data(); h->version=OFP_VERSION; h->type=OFPT_ECHO_REPLY;
    h->length=htobe16_u(buf.size()); h->xid=req->xid;
    if(len) memcpy(buf.data()+sizeof(ofp_header), payload, len);
    send_all(fd, buf.data(), buf.size());
  }
  void send_barrier(int fd){
    ofp_header h{}; h.version=OFP_VERSION; h.type=OFPT_BARRIER_REQUEST;
    h.length=htobe16_u(sizeof(h)); h.xid=htobe32_u(xid++);
    send_all(fd,&h,sizeof(h));
  }

  static std::vector<uint8_t> build_lldp_eth(uint64_t chassis_id, uint16_t port_no){
    std::vector<uint8_t> f;
    auto push16=[&](uint16_t v){ f.push_back(uint8_t(v>>8)); f.push_back(uint8_t(v)); };
    uint8_t dst[6]={0x01,0x80,0xc2,0x00,0x00,0x0e};
    uint8_t src[6]={0x02,0x00,0x00,0x00,0x00,0x01};
    f.insert(end(f), dst,dst+6); f.insert(end(f), src,src+6); push16(0x88cc);
    { std::vector<uint8_t> v; v.push_back(7);
      for(int i=7;i>=0;--i) v.push_back(uint8_t((chassis_id>>(8*i))&0xff));
      uint16_t tl=(1<<9)|(uint16_t(v.size())&0x1ff); push16(tl); f.insert(end(f),begin(v),end(v));
    }
    { std::vector<uint8_t> v; v.push_back(5); v.push_back(uint8_t(port_no>>8)); v.push_back(uint8_t(port_no));
      uint16_t tl=(2<<9)|(uint16_t(v.size())&0x1ff); push16(tl); f.insert(end(f),begin(v),end(v));
    }
    { push16((3<<9)|2); push16(120); } // TTL
    { push16(0); } // End TLV
    if(f.size()<14+46) f.resize(14+46,0);
    return f;
  }

  void packet_out_lldp(int fd, uint16_t out_port, uint64_t dpid){
    auto frame=build_lldp_eth(dpid,out_port);
    ofp_action_output act{htobe16_u(OFPAT_OUTPUT),htobe16_u(sizeof(ofp_action_output)),
                          htobe16_u(out_port),htobe16_u(0)};
    ofp_packet_out po{};
    po.header.version=OFP_VERSION; po.header.type=OFPT_PACKET_OUT;
    po.buffer_id=htobe32_u(0xffffffff); po.in_port=htobe16_u(OFPP_NONE);
    po.actions_len=htobe16_u(sizeof(act));
    po.header.length=htobe16_u(sizeof(po)+sizeof(act)+frame.size());
    po.header.xid=htobe32_u(xid++);
    std::vector<uint8_t> buf(sizeof(po)+sizeof(act)+frame.size());
    memcpy(buf.data(),&po,sizeof(po));
    memcpy(buf.data()+sizeof(po),&act,sizeof(act));
    memcpy(buf.data()+sizeof(po)+sizeof(act),frame.data(),frame.size());
    send_all(fd,buf.data(),buf.size());
  }

  void send_packet_out_with_buffer(int fd, uint32_t buffer_id, uint16_t in_port, uint16_t out_port){
    ofp_action_output act{htobe16_u(OFPAT_OUTPUT),htobe16_u(sizeof(ofp_action_output)),
                          htobe16_u(out_port),htobe16_u(0)};
    ofp_packet_out po{};
    po.header.version=OFP_VERSION; po.header.type=OFPT_PACKET_OUT;
    po.buffer_id=htobe32_u(buffer_id);
    po.in_port = htobe16_u(in_port);
    po.actions_len = htobe16_u(sizeof(act));
    po.header.length = htobe16_u(sizeof(po)+sizeof(act));
    po.header.xid = htobe32_u(xid++);
    std::vector<uint8_t> buf(sizeof(po)+sizeof(act));
    memcpy(buf.data(), &po, sizeof(po));
    memcpy(buf.data()+sizeof(po), &act, sizeof(act));
    send_all(fd, buf.data(), buf.size());
  }

  void send_port_stats_req(int fd, uint16_t port = 0xffff){
    std::vector<uint8_t> buf(sizeof(ofp_stats_request)+sizeof(ofp_port_stats_request));
    auto* req=(ofp_stats_request*)buf.data();
    req->header.version=OFP_VERSION; req->header.type=OFPT_STATS_REQUEST;
    req->type=htobe16_u(OFPST_PORT); req->flags=0; req->header.xid=htobe32_u(xid++);
    req->header.length=htobe16_u(buf.size());
    auto* pr=(ofp_port_stats_request*)req->body; pr->port_no=htobe16_u(port); memset(pr->pad,0,sizeof(pr->pad));
    send_all(fd,buf.data(),buf.size());
  }

  // events
  void on_features_reply(int fd, const ofp_switch_features* fr){
    uint64_t dpid = ntohl(uint32_t(fr->datapath_id)) | (uint64_t(ntohl(uint32_t(fr->datapath_id>>32)))<<32);
    std::lock_guard<std::mutex> lk(mtx);
    auto& s = sw[fd];
    s.dpid = dpid;
    if(sw_index_to_fd.empty()) sw_index_to_fd[1]=fd;
    else {
      int maxIdx = sw_index_to_fd.rbegin()->first;
      bool exists=false; for(auto& kv: sw_index_to_fd) if(kv.second==fd) exists=true;
      if(!exists) sw_index_to_fd[maxIdx+1]=fd;
    }
    // 設定 miss_send_len，否則 PACKET_IN 不會帶 payload
    send_set_config(fd, /*flags*/0, /*miss*/0xffff);
    send_get_config_req(fd);
  }

  void on_stats_reply(int fd, const ofp_stats_reply* r){
    if(ntohs(r->type)!=OFPST_PORT) return;
    size_t body_len = ntohs(r->header.length) - sizeof(ofp_stats_reply);
    const uint8_t* p = r->body; size_t off=0;
    std::lock_guard<std::mutex> lk(mtx);
    while(off + sizeof(ofp_port_stats) <= body_len){
      auto* ps = (const ofp_port_stats*)(p+off);
      uint16_t port = ntohs(ps->port_no);
      sw[fd].last_ps[port] = *ps;
      off += sizeof(ofp_port_stats);
    }
  }

  void on_packet_in(int fd, const ofp_packet_in* pi){
    const uint8_t* frame = pi->data;
    if(ntohs(pi->total_len) < 14) return;
    const uint8_t* dst = frame+0;
    const uint8_t* src = frame+6;
    uint16_t eth_type = (frame[12]<<8) | frame[13];
    uint16_t in_port = ntohs(pi->in_port);

    // 學習來源 MAC -> in_port
    {
      std::lock_guard<std::mutex> lk(mtx);
      sw[fd].mac2port[mac_to_key(src)] = in_port;
    }

    // 查目的 MAC 是否已知
    int out_port = -1;
    {
      std::lock_guard<std::mutex> lk(mtx);
      auto it = sw[fd].mac2port.find(mac_to_key(dst));
      if(it != sw[fd].mac2port.end()) out_port = it->second;
    }

    // 已知 → 安裝單向 flow 並轉送；未知 → FLOOD
    if(out_port > 0 && out_port != in_port){
      // 安裝 flow: match in_port + dl_dst，動作 output:out_port
      ofp_flow_mod fm{}; fm.header.version=OFP_VERSION; fm.header.type=OFPT_FLOW_MOD;
      fm.cookie      = htobe64_u(0x1ULL);
      fm.command     = htobe16_u(OFPFC_ADD);
      fm.idle_timeout= htobe16_u(30);
      fm.hard_timeout= htobe16_u(0);
      fm.priority    = htobe16_u(100);
      fm.buffer_id   = pi->buffer_id;                // 讓 switch 直接轉出這個封包
      fm.out_port    = htobe16_u(0xffff);
      fm.flags       = htobe16_u(0);

      memset(&fm.match, 0, sizeof(fm.match));
      fm.match.in_port = htobe16_u(in_port);
      fm.match.dl_type = 0; // 只匹配 L2
      memcpy(fm.match.dl_dst, dst, 6);
      uint32_t wc = OFPFW_DL_VLAN | OFPFW_DL_SRC | OFPFW_DL_VLAN_PCP |
                    OFPFW_DL_TYPE | OFPFW_NW_TOS | OFPFW_NW_PROTO |
                    OFPFW_TP_SRC  | OFPFW_TP_DST; // 只指定 in_port + dl_dst
      fm.match.wildcards = htonl(wc);

      ofp_action_output act{htobe16_u(OFPAT_OUTPUT), htobe16_u(sizeof(ofp_action_output)),
                            htobe16_u(uint16_t(out_port)), htobe16_u(0)};
      const size_t len = sizeof(fm) + sizeof(act);
      fm.header.length = htobe16_u(len);
      fm.header.xid    = htobe32_u(xid++);
      std::vector<uint8_t> buf(len);
      memcpy(buf.data(), &fm, sizeof(fm));
      memcpy(buf.data()+sizeof(fm), &act, sizeof(act));
      send_all(fd, buf.data(), buf.size());
    } else {
      // 未知 → FLOOD（使用 buffer_id，避免攜帶 payload）
      send_packet_out_with_buffer(fd, pi->buffer_id, in_port, OFPP_FLOOD);
    }
  }

  void loop(uint16_t port){
    listen_fd = listen_on(port);
    if(listen_fd<0) return;
    running = true;

    auto last_lldp  = std::chrono::steady_clock::now();
    auto last_stats = std::chrono::steady_clock::now();

    while(running){
      fd_set rfds; FD_ZERO(&rfds); int maxfd=listen_fd; FD_SET(listen_fd,&rfds);
      {
        std::lock_guard<std::mutex> lk(mtx);
        for(auto& kv: sw){ FD_SET(kv.first,&rfds); maxfd=max(maxfd, kv.first); }
      }
      timeval tv{1,0};
      int rv = select(maxfd+1, &rfds, nullptr, nullptr, &tv);
      if(rv<0){ if(errno==EINTR) continue; perror("select"); break; }

      if(FD_ISSET(listen_fd,&rfds)){
        int cfd = accept(listen_fd,nullptr,nullptr);
        if(cfd>=0){
          std::lock_guard<std::mutex> lk(mtx);
          sw[cfd] = SwCtx{cfd,0,{}};
          send_hello(cfd);
          send_features_req(cfd);
          // 提前設 config，某些 switch 會在 features 之前也允許
          send_set_config(cfd, 0, 0xffff);
        }
      }

      std::vector<int> closed;

      // ---- 收包（修正：把 header+body 併成一塊解析）----
      {
        std::lock_guard<std::mutex> lk(mtx);
        for(auto& kv: sw){
          int fd=kv.first;
          if(!FD_ISSET(fd,&rfds)) continue;

          ofp_header h{};
          ssize_t n=recv(fd,&h,sizeof(h),MSG_WAITALL);
          if(n<=0){ closed.push_back(fd); continue; }
          if(n != (ssize_t)sizeof(h) || h.version!=OFP_VERSION){ closed.push_back(fd); continue; }

          uint16_t mlen = ntohs(h.length);
          if(mlen < sizeof(ofp_header)){ closed.push_back(fd); continue; }
          std::vector<uint8_t> full(mlen);
          memcpy(full.data(), &h, sizeof(h));
          if(mlen > sizeof(h)){
            ssize_t m=recv(fd, full.data()+sizeof(h), mlen - sizeof(h), MSG_WAITALL);
            if(m<=0){ closed.push_back(fd); continue; }
          }

          auto* base = (const ofp_header*)full.data();
          switch(base->type){
            case OFPT_HELLO: break;
            case OFPT_ECHO_REQUEST: {
              const uint8_t* payload = full.data()+sizeof(ofp_header);
              size_t plen = mlen - sizeof(ofp_header);
              send_echo_reply(fd, base, payload, plen);
              break;
            }
            case OFPT_FEATURES_REPLY:
              on_features_reply(fd, (const ofp_switch_features*)base);
              break;
            case OFPT_PACKET_IN:
              on_packet_in(fd, (const ofp_packet_in*)base);
              break;
            case OFPT_STATS_REPLY:
              on_stats_reply(fd,(const ofp_stats_reply*)base);
              break;
            default: break;
          }
        }
      }

      if(!closed.empty()){
        std::lock_guard<std::mutex> lk(mtx);
        for(int fd: closed){ close(fd); sw.erase(fd);
          for(auto it=sw_index_to_fd.begin(); it!=sw_index_to_fd.end();){
            if(it->second==fd) it=sw_index_to_fd.erase(it); else ++it;
          }
        }
      }

      auto now = std::chrono::steady_clock::now();
      if(now - last_lldp > std::chrono::seconds(2)){
        std::lock_guard<std::mutex> lk(mtx);
        for(auto& kv: sw){
          int fd=kv.first; uint64_t dpid=kv.second.dpid?kv.second.dpid:0xdeadbeef;
          for(int p=1;p<=4;++p) packet_out_lldp(fd, uint16_t(p), dpid);
        }
        last_lldp = now;
      }
      if(now - last_stats > std::chrono::seconds(3)){
        std::lock_guard<std::mutex> lk(mtx);
        for(auto& kv: sw) send_port_stats_req(kv.first, 0xffff);
        last_stats = now;
      }
    }
  }
};

// 唯一實例
namespace { OFControllerImpl g_impl; }

// =============================
// OFController public methods
// =============================
bool OFController::start(uint16_t port){
  if(g_impl.running) return true;
  g_impl.loop_thread = std::thread([port]{ g_impl.loop(port); });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  return true;
}

void OFController::send_lldp(int swid, int out_port){
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if(it==g_impl.sw_index_to_fd.end()) return;
  int fd = it->second;
  uint64_t dpid = g_impl.sw[fd].dpid? g_impl.sw[fd].dpid : 0xdeadbeef;
  g_impl.packet_out_lldp(fd, uint16_t(out_port), dpid);
}

std::map<LinkId,PortStats> OFController::poll_port_stats(){
  std::map<LinkId,PortStats> out;
  {
    std::lock_guard<std::mutex> lk(g_impl.mtx);
    for(auto& kv: g_impl.sw) g_impl.send_port_stats_req(kv.first, 0xffff);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  std::lock_guard<std::mutex> lk(g_impl.mtx);
  for(auto& idx : g_impl.sw_index_to_fd){
    int swid = idx.first; int fd = idx.second;
    auto it = g_impl.sw.find(fd); if(it==g_impl.sw.end()) continue;
    for(const auto& p : it->second.last_ps){
      int port = p.first;
      const auto& ps = p.second;
      PortStats s{};
      s.rx_bytes = be64toh(*(const uint64_t*)&ps.rx_bytes);
      s.tx_bytes = be64toh(*(const uint64_t*)&ps.tx_bytes);
      s.speedMbps = 0;
      out[{swid, port}] = s;
    }
  }
  return out;
}

void OFController::flow_mod(int swid,
                            const std::string& match,
                            const std::string& actions,
                            int priority,
                            bool add,
                            std::optional<uint16_t> idle_timeout,
                            std::optional<uint16_t> hard_timeout,
                            std::optional<uint64_t> cookie)
{
  auto parse_ip = [](const std::string& s)->uint32_t {
    in_addr a{}; if(inet_aton(s.c_str(), &a)==0) return 0; return a.s_addr;
  };

  uint16_t in_port=0, out_port=0;
  uint8_t  proto=0;
  uint32_t src=0, dst=0;
  std::optional<uint16_t> sport, dport;

  auto eat = [&](const std::string& k, std::string* v)->bool{
    auto pos = match.find(k);
    if (pos == std::string::npos) return false;
    auto p2 = match.find_first_of(", ", pos + k.size());
    *v = match.substr(pos + k.size(), p2 == std::string::npos ? std::string::npos : (p2 - (pos + k.size())));
    return true;
  };

  std::string v;
  if (eat("in=",   &v) || eat("in_port=", &v)) in_port = uint16_t(std::stoi(v));
  if (eat("src=",  &v) || eat("nw_src=",  &v)) src = parse_ip(v);
  if (eat("dst=",  &v) || eat("nw_dst=",  &v)) dst = parse_ip(v);
  if (eat("proto=",&v) || eat("nw_proto=",&v)) proto = uint8_t(std::stoi(v));
  if (eat("sport=",&v) || eat("tp_src=",  &v)) if(v!="-") sport = uint16_t(std::stoi(v));
  if (eat("dport=",&v) || eat("tp_dst=",  &v)) if(v!="-") dport = uint16_t(std::stoi(v));
  if (actions.rfind("output:",0) == 0)       out_port = uint16_t(std::stoi(actions.substr(7)));
  else if (actions.rfind("output=",0) == 0)  out_port = uint16_t(std::stoi(actions.substr(7)));

  auto send_flow = [&](int fd){
    ofp_flow_mod fm{}; fm.header.version=OFP_VERSION; fm.header.type=OFPT_FLOW_MOD;
    fm.cookie      = htobe64_u(cookie.value_or(0x1234ULL));
    fm.command     = htobe16_u(add ? OFPFC_ADD : OFPFC_DELETE_STRICT);
    fm.idle_timeout= htobe16_u(idle_timeout.value_or(0));
    fm.hard_timeout= htobe16_u(hard_timeout.value_or(0));
    fm.priority    = htobe16_u(uint16_t(priority));
    fm.buffer_id   = htobe32_u(0xffffffff);
    fm.out_port    = htobe16_u(0xffff);
    fm.flags       = htobe16_u(0);

    memset(&fm.match, 0, sizeof(fm.match));
    fm.match.in_port = htobe16_u(in_port);
    fm.match.dl_type = htobe16_u(0x0800);
    fm.match.nw_proto= proto;
    memcpy(fm.match.nw_src, &src, 4);
    memcpy(fm.match.nw_dst, &dst, 4);
    fm.match.tp_src = htobe16_u(sport.value_or(0));
    fm.match.tp_dst = htobe16_u(dport.value_or(0));

    uint32_t wc = OFPFW_DL_VLAN | OFPFW_DL_SRC | OFPFW_DL_DST | OFPFW_DL_VLAN_PCP;
    if (in_port==0) wc |= OFPFW_IN_PORT;
    if (proto==0)   wc |= OFPFW_NW_PROTO;
    if (!sport)     wc |= OFPFW_TP_SRC;
    if (!dport)     wc |= OFPFW_TP_DST;
    fm.match.wildcards = htonl(wc);

    ofp_action_output act{htobe16_u(OFPAT_OUTPUT), htobe16_u(sizeof(ofp_action_output)),
                          htobe16_u(out_port), htobe16_u(0)};
    const size_t len = sizeof(fm) + (add ? sizeof(act) : 0);
    fm.header.length = htobe16_u(len);
    fm.header.xid    = htobe32_u(g_impl.xid++);

    std::vector<uint8_t> buf(len);
    memcpy(buf.data(), &fm, sizeof(fm));
    if (add) memcpy(buf.data()+sizeof(fm), &act, sizeof(act));
    OFControllerImpl::send_all(fd, buf.data(), buf.size());
    g_impl.send_barrier(fd);
  };

  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if (it == g_impl.sw_index_to_fd.end()) return;
  send_flow(it->second);
}

void OFController::packet_out(int swid, int out_port, const uint8_t* eth, size_t len)
{
  if (!eth || len < 14) return; // 需為完整 L2 幀
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if (it == g_impl.sw_index_to_fd.end()) return;
  int fd = it->second;

  ofp_action_output act{htobe16_u(OFPAT_OUTPUT), htobe16_u(sizeof(ofp_action_output)),
                        htobe16_u(uint16_t(out_port)), htobe16_u(0)};
  ofp_packet_out po{};
  po.header.version=OFP_VERSION; po.header.type=OFPT_PACKET_OUT;
  po.buffer_id=htobe32_u(0xffffffff);
  po.in_port = htobe16_u(0xffff);
  po.actions_len = htobe16_u(sizeof(act));
  po.header.length = htobe16_u(sizeof(po)+sizeof(act)+len);
  po.header.xid = htobe32_u(g_impl.xid++);

  std::vector<uint8_t> buf(sizeof(po)+sizeof(act)+len);
  memcpy(buf.data(), &po, sizeof(po));
  memcpy(buf.data()+sizeof(po), &act, sizeof(act));
  memcpy(buf.data()+sizeof(po)+sizeof(act), eth, len);
  OFControllerImpl::send_all(fd, buf.data(), buf.size());
}

void OFController::barrier(int swid){
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if(it==g_impl.sw_index_to_fd.end()) return;
  g_impl.send_barrier(it->second);
}

std::map<int, PortStats> OFController::poll_port_stats(int swid){
  std::map<int, PortStats> out;
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if(it==g_impl.sw_index_to_fd.end()) return out;
  int fd = it->second;

  g_impl.send_port_stats_req(fd, 0xffff);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  auto sit = g_impl.sw.find(fd);
  if (sit == g_impl.sw.end()) return out;
  for (const auto& kv : sit->second.last_ps) {
    const int port = kv.first;
    const auto& ps = kv.second;
    PortStats s{};
    s.rx_bytes  = be64toh(*(const uint64_t*)&ps.rx_bytes);
    s.tx_bytes  = be64toh(*(const uint64_t*)&ps.tx_bytes);
    s.speedMbps = 0;
    out[port] = s;
  }
  return out;
}

std::string OFController::ip_match(int in_port,
                                   const std::string& src, const std::string& dst,
                                   int ip_proto,
                                   std::optional<int> tp_src,
                                   std::optional<int> tp_dst)
{
  std::ostringstream ss;
  ss << "in=" << in_port << ",ip,src=" << src << ",dst=" << dst << ",proto=" << ip_proto
     << ",sport=" << (tp_src ? std::to_string(*tp_src) : "-")
     << ",dport=" << (tp_dst ? std::to_string(*tp_dst) : "-");
  return ss.str();
}

// =============================
// Extra public methods needed by TopoViewer / Monitor
// =============================
void OFController::on_lldp(OnLLDP cb) { std::lock_guard<std::mutex> lk(mtx_); cb_lldp_ = std::move(cb); }
void OFController::set_lldp_period(std::chrono::milliseconds p) { std::lock_guard<std::mutex> lk(mtx_); lldp_period_ = p; }
void OFController::set_stats_period(std::chrono::milliseconds p){ std::lock_guard<std::mutex> lk(mtx_); stats_period_ = p; }

std::vector<int> OFController::switch_ids() const {
  std::vector<int> ids;
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  ids.reserve(g_impl.sw_index_to_fd.size());
  for (const auto& kv : g_impl.sw_index_to_fd) ids.push_back(kv.first);
  return ids;
}

std::vector<int> OFController::ports_of(int swid) const {
  std::vector<int> ports;
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if (it == g_impl.sw_index_to_fd.end()) return ports;
  int fd = it->second;
  auto sit = g_impl.sw.find(fd);
  if (sit == g_impl.sw.end()) return ports;
  ports.reserve(sit->second.last_ps.size());
  for (const auto& kv : sit->second.last_ps) ports.push_back(kv.first);
  std::sort(ports.begin(), ports.end());
  return ports;
}

std::optional<SwitchInfo> OFController::switch_info(int swid) const {
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if (it == g_impl.sw_index_to_fd.end()) return std::nullopt;
  int fd = it->second;
  auto sit = g_impl.sw.find(fd);
  if (sit == g_impl.sw.end()) return std::nullopt;

  SwitchInfo info;
  info.swid = swid;
  info.dpid = sit->second.dpid;
  info.connected = true;
  for (const auto& kv : sit->second.last_ps) {
    PortInfo pi{};
    pi.port_no = kv.first;
    pi.up = true;
    pi.curr_speedMbps = 0;
    const auto& ps = kv.second;
    pi.last.rx_bytes = be64toh(*(const uint64_t*)&ps.rx_bytes);
    pi.last.tx_bytes = be64toh(*(const uint64_t*)&ps.tx_bytes);
    pi.last.speedMbps = 0;
    info.ports[pi.port_no] = pi;
  }
  return info;
}

void OFController::port_mod(int swid, int port_no, bool up, int speedMbps){
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  auto it = g_impl.sw_index_to_fd.find(swid);
  if(it==g_impl.sw_index_to_fd.end()) return;
  int fd = it->second;

  ofp_port_mod pm{}; pm.header.version=OFP_VERSION; pm.header.type=OFPT_PORT_MOD;
  pm.port_no=htobe16_u(uint16_t(port_no));
  memset(pm.hw_addr, 0, sizeof(pm.hw_addr));
  pm.config = up ? 0 : htonl(OFPPC_PORT_DOWN);
  pm.mask   = htonl(OFPPC_PORT_DOWN);
  pm.advertise = htonl(advertise_mask_for_speed(speedMbps));
  pm.header.length=htobe16_u(sizeof(pm));
  pm.header.xid=htobe32_u(g_impl.xid++);
  OFControllerImpl::send_all(fd,&pm,sizeof(pm));
  g_impl.send_barrier(fd);
}

void OFController::stop(){
  if(!g_impl.running) return;
  g_impl.running=false;
  if(g_impl.listen_fd>=0){ close(g_impl.listen_fd); g_impl.listen_fd=-1; }
  if(g_impl.loop_thread.joinable()) g_impl.loop_thread.join();
  std::lock_guard<std::mutex> lk(g_impl.mtx);
  for(auto& kv : g_impl.sw){ close(kv.first); }
  g_impl.sw.clear(); g_impl.sw_index_to_fd.clear();
}

