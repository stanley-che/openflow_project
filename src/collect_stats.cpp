// g++ -O2 -std=c++17 collect_stats.cpp -o collect_stats
// 用法(需 sudo)：sudo ./collect_stats --duration 30 --interval 1 --capacity 1e9 --topo NSFNET --sdn_pct 0.4 --flows 6 --out results/nsfnet_40pct_6flows.csv
#include <bits/stdc++.h>
using namespace std;

static string run_cmd(const string& cmd) {
    array<char, 8192> buf{};
    string data;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return data;
    while (fgets(buf.data(), (int)buf.size(), pipe)) data += buf.data();
    pclose(pipe);
    return data;
}

struct PortStat { string name; long long rx=0, tx=0; };
using BridgeSnapshot = unordered_map<int, PortStat>;

static vector<string> list_bridges() {
    string out = run_cmd("sudo ovs-vsctl list-br");
    vector<string> brs;
    string line; stringstream ss(out);
    while (getline(ss, line)) if(!line.empty()) brs.push_back(line);
    return brs;
}

// 兼容不同版本 ovs-ofctl dump-ports 的輸出
static BridgeSnapshot dump_ports(const string& br) {
    BridgeSnapshot snap;
    string cmd = "sudo ovs-ofctl -O OpenFlow10 dump-ports " + br + " 2>/dev/null";
    string out = run_cmd(cmd);

    // 先試新樣式: "... rx bytes:<num> ... tx bytes:<num>"
    {
        regex re(R"((\d+)\(([^)]+)\).*?rx bytes[: ]+(\d+).*?tx bytes[: ]+(\d+))",
                 regex::icase | regex::optimize | regex::nosubs);
        sregex_iterator it(out.begin(), out.end(), re), ed;
        for (; it != ed; ++it) {
            int pid = stoi((*it)[1].str());
            string name = (*it)[2].str();
            long long rx = stoll((*it)[3].str());
            long long tx = stoll((*it)[4].str());
            snap[pid] = {name, rx, tx};
        }
        if (!snap.empty()) return snap;
    }

    // 備援：舊樣式逐行找 pid/name + 後續行的 rx/tx bytes
    {
        vector<string> lines;
        { string l; stringstream ss(out); while (getline(ss,l)) lines.push_back(l); }
        int curPid=-1; string curName; long long rx=-1, tx=-1;
        auto flush = [&](){
            if(curPid>=0 && rx>=0 && tx>=0) snap[curPid] = {curName, rx, tx};
            curPid=-1; curName=""; rx=-1; tx=-1;
        };
        regex reHdr(R"(^\s*(\d+)\(([^)]+)\):)");
        for (auto& l: lines) {
            smatch m;
            if (regex_search(l, m, reHdr)) {
                flush();
                curPid = stoi(m[1].str());
                curName = m[2].str();
                continue;
            }
            auto low = l; for (auto& c: low) c=::tolower(c);
            if (low.find("rx bytes")!=string::npos) {
                auto pos = low.find("bytes"); if (pos!=string::npos) {
                    auto tail = low.substr(pos);
                    smatch m2; if (regex_search(tail, m2, regex(R"((\d+))"))) rx = stoll(m2[1].str());
                }
            }
            if (low.find("tx bytes")!=string::npos) {
                auto pos = low.find("bytes"); if (pos!=string::npos) {
                    auto tail = low.substr(pos);
                    smatch m2; if (regex_search(tail, m2, regex(R"((\d+))"))) tx = stoll(m2[1].str());
                }
            }
        }
        flush();
    }
    return snap;
}

static void sleep_seconds(double s) {
    using namespace std::chrono;
    this_thread::sleep_for(duration<double>(s));
}

int main(int argc, char** argv) {
    // 參數
    int duration = 30;
    double interval = 1.0;
    double capacity = 1e9; // bps per-port
    string topo="NSFNET", out="results/nsfnet_40pct_6flows.csv";
    double sdn_pct = 0.4;
    int flows = 6;

    // 解析命令列
    for (int i=1;i<argc;i++){
        string a=argv[i];
        auto need = [&](string k)->bool{ return a==k && i+1<argc; };
        if (need("--duration")) duration=stoi(argv[++i]);
        else if (need("--interval")) interval=stod(argv[++i]);
        else if (need("--capacity")) capacity=stod(argv[++i]);
        else if (need("--topo")) topo=argv[++i];
        else if (need("--sdn_pct")) sdn_pct=stod(argv[++i]);
        else if (need("--flows")) flows=stoi(argv[++i]);
        else if (need("--out")) out=argv[++i];
        else if (a=="-h"||a=="--help"){
            cerr<<"Usage: sudo "<<argv[0]<<" [--duration 30] [--interval 1] [--capacity 1e9] [--topo NSFNET] [--sdn_pct 0.4] [--flows 6] [--out results/x.csv]\n";
            return 0;
        }
    }

    // 建立資料夾
    {
        string mk = "mkdir -p " + string(out.find('/')!=string::npos ? out.substr(0,out.find_last_of('/')) : ".");
        system(mk.c_str());
    }

    // 列出所有 bridge
    auto brs = list_bridges();
    if (brs.empty()) { cerr<<"[fatal] no OVS bridges.\n"; return 1; }

    // 基準快照
    unordered_map<string, BridgeSnapshot> base;
    for (auto& b: brs) base[b] = dump_ports(b);

    sleep_seconds(interval);

    vector<double> samples;
    auto t0 = chrono::steady_clock::now();
    while (chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now()-t0).count() < duration) {
        unordered_map<string, BridgeSnapshot> now;
        vector<double> utils;
        for (auto& b: brs) {
            now[b] = dump_ports(b);
            for (auto& [pid, cur] : now[b]) {
                auto it = base[b].find(pid);
                if (it==base[b].end()) continue;
                const auto& prev = it->second;
                if (cur.name=="LOCAL" || cur.name=="local") continue;
                long long drx = max(0LL, cur.rx - prev.rx);
                long long dtx = max(0LL, cur.tx - prev.tx);
                double bps = double(drx + dtx) * 8.0 / interval;
                double util = bps / capacity;
                utils.push_back(util);
            }
        }
        double max_u = utils.empty() ? 0.0 : *max_element(utils.begin(), utils.end());
        samples.push_back(max_u);
        base.swap(now);
        sleep_seconds(interval);
    }

    double avg = 0.0;
    if (!samples.empty()) avg = accumulate(samples.begin(), samples.end(), 0.0) / samples.size();

    // 輸出 CSV
    ofstream fo(out);
    fo<<"topo,sdn_pct,flows,duration,avg_max_link_util\n";
    fo<<topo<<","<<sdn_pct<<","<<flows<<","<<duration<<","<<fixed<<setprecision(6)<<avg<<"\n";
    fo.close();

    cout<<"[OK] wrote "<<out<<", avg_max_link_util="<<fixed<<setprecision(4)<<avg<<"\n";
    return 0;
}

