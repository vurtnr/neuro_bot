# 4G/5G 网络状态与 GPS 信息

string interface_name   # 网卡名称, e.g. "wwan0"
bool is_connected       # 是否联网
string ip_address       # IP 地址
string operator_name    # 运营商, e.g. "CMCC", "China Mobile"
int8 signal_strength    # CSQ 信号值 (0-31, 99=未知)
string tech             # "LTE", "NR", "WCDMA"

# GPS 信息 (如果未定位则为 0.0)
float64 latitude
float64 longitude