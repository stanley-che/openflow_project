// g++ -O2 -std=c++17 compute_energy.cpp -o compute_energy
// 用法：./compute_energy --in results/nsfnet_40pct_6flows.csv --out results/nsfnet_40pct_6flows_energy.csv
#include <bits/stdc++.h>
using namespace std;

// 簡單功耗模型（示意）：把「平均最大利用率」映射成檔位功耗
static double slot_power(double util){
    if (util >= 0.80) return 1.00; // 100%
    if (util >= 0.50) return 0.60; // 60%
    if (util >= 0.20) return 0.40; // 40%
    if (util <= 0.01) return 0.10; // 幾乎閒置
    return 0.40;
}

int main(int argc, char** argv){
    string in="results/nsfnet_40pct_6flows.csv";
    string out="results/nsfnet_40pct_6flows_energy.csv";
    for (int i=1;i<argc;i++){
        string a=argv[i];
        if ((a=="--in"||a=="--input") && i+1<argc) in=argv[++i];
        else if ((a=="--out"||a=="--output") && i+1<argc) out=argv[++i];
        else if (a=="-h"||a=="--help"){
            cerr<<"Usage: "<<argv[0]<<" --in <csv> --out <csv>\n"; return 0;
        }
    }

    ifstream fi(in);
    if(!fi) { cerr<<"[fatal] cannot open "<<in<<"\n"; return 1; }

    string header; getline(fi, header);
    string row; getline(fi, row);
    if(row.empty()){ cerr<<"[fatal] empty data row\n"; return 1; }

    // 解析 CSV（簡單 split）
    vector<string> cols; {
        string tmp; stringstream ss(row);
        while (getline(ss, tmp, ',')) cols.push_back(tmp);
    }
    if (cols.size()<5){ cerr<<"[fatal] bad csv columns\n"; return 1; }

    string topo=cols[0];
    double sdn_pct = stod(cols[1]);
    int flows = stoi(cols[2]);
    int duration = stoi(cols[3]);
    double util = stod(cols[4]);

    double p_baseline = 1.00;
    double p_curr = slot_power(util);
    double saving = (p_baseline - p_curr)/p_baseline * 100.0;

    // 輸出
    {
        size_t slash = out.find_last_of('/');
        if (slash!=string::npos) {
            string mk = "mkdir -p " + out.substr(0, slash);
            system(mk.c_str());
        }
    }
    ofstream fo(out);
    fo<<"topo,sdn_pct,flows,duration,avg_max_link_util,power_saving_pct\n";
    fo<<topo<<","<<sdn_pct<<","<<flows<<","<<duration<<","<<fixed<<setprecision(6)<<util<<","<<setprecision(2)<<saving<<"\n";
    fo.close();
    cout<<"[OK] wrote "<<out<<", power_saving_pct="<<fixed<<setprecision(2)<<saving<<"%\n";
    return 0;
}

