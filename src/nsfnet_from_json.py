# nsfnet_from_json.py  (robust 版)
# 用法：
#   sudo mn --custom nsfnet_from_json.py --topo nsfnet,\
#            json=config/NSFNET.json,addhosts=True \
#            --switch ovsk,protocols=OpenFlow10 \
#            --controller=remote,ip=127.0.0.1,port=6633

from mininet.topo import Topo
import json

def _clean(s):
    s = str(s)
    return ''.join(ch for ch in s if ch.isalnum() or ch == '_')

def _node_id(n):
    # n 可能是 dict / str / int
    if isinstance(n, dict):
        return str(n.get('id', n.get('name', n.get('node', n))))
    return str(n)

def _edge_uv(e):
    # e 可能是 dict（src/dst 或 u/v）或 list/tuple 兩元素
    if isinstance(e, dict):
        u = e.get('src', e.get('u'))
        v = e.get('dst', e.get('v'))
        if u is None or v is None:
            raise KeyError('Edge dict needs "src/dst" or "u/v": %r' % e)
        return str(u), str(v)
    if isinstance(e, (list, tuple)) and len(e) == 2:
        return str(e[0]), str(e[1])
    raise TypeError('Edge must be dict or 2-tuple/list: %r' % e)

class JsonTopo(Topo):
    """
    可接受的 JSON 例子：
    {
      "nodes": ["1","2","3"]
      "links": [["1","2"],["2","3"]]
    }
    或
    {
      "nodes": [{"id":"1"},{"name":"2"}],
      "links": [{"src":"1","dst":"2"}, {"u":"2","v":"3"}]
    }
    """
    def __init__(self, json='config/NSFNET.json', addhosts=True, **opts):
        self.json_path = json
        self.add_hosts = (str(addhosts).lower() != 'false')
        super(JsonTopo, self).__init__(**opts)

    def build(self):
        with open(self.json_path, 'r') as f:
            topo = json.load(f)

        nodes = topo.get('nodes', topo.get('Vertices', topo.get('routers', [])))
        links = topo.get('links', topo.get('Edges', topo.get('edges', [])))

        if not nodes:
            raise ValueError('JSON has no "nodes" (or Vertices/routers)')
        if not links:
            raise ValueError('JSON has no "links" (or Edges/edges)')

        # 建 switch
        sw = {}
        for n in nodes:
            nid_raw = _node_id(n)
            nid = _clean(nid_raw)
            if not nid:
                raise ValueError('Invalid node id: %r' % (nid_raw,))
            sw_name = 's' + nid if not nid.startswith('s') else nid
            sw[nid_raw] = self.addSwitch(sw_name)

        # 每個 switch 掛個 host（可關）
        if self.add_hosts:
            for nid_raw, sid in sw.items():
                nid = _clean(nid_raw)
                hname = 'h' + nid if not nid.startswith('h') else nid
                h = self.addHost(hname)
                self.addLink(h, sid)

        # 建連線
        for e in links:
            u_raw, v_raw = _edge_uv(e)
            if u_raw not in sw or v_raw not in sw:
                # 兼容：若節點陣列是純數字，edge 又是數字，會直接對上
                # 否則提醒檢查 nodes/links 是否用同一種 id
                raise KeyError('Link references unknown node(s): %r -> %r' % (u_raw, v_raw))
            self.addLink(sw[u_raw], sw[v_raw])

def nsfnet(**kwargs):
    return JsonTopo(**kwargs)

topos = {'nsfnet': nsfnet}

