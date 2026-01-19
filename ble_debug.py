import asyncio
import sys
from bleak import BleakScanner, BleakClient

async def scan_devices():
    print("📡 正在扫描附近的蓝牙设备 (5秒)...")
    devices = await BleakScanner.discover(timeout=5.0)
    
    if not devices:
        print("❌ 未发现任何设备。请检查蓝牙是否开启，或是否有权限。")
        return

    print(f"✅ 扫描完成，发现 {len(devices)} 个设备：")
    print("-" * 40)
    for i, d in enumerate(devices):
        # 过滤掉名字为空的设备，方便查看
        name = d.name if d.name else "Unknown"
        print(f"[{i}] MAC: {d.address} | Name: {name} | RSSI: {d.rssi}")
    print("-" * 40)
    return devices

async def inspect_device(address):
    print(f"\n🔗 正在尝试连接到 {address} ...")
    try:
        async with BleakClient(address) as client:
            connected = await client.is_connected()
            print(f"✅ 连接状态: {'成功' if connected else '失败'}")
            
            if connected:
                print("\n📂 服务与特征值列表 (UUID):")
                print("=" * 60)
                for service in client.services:
                    print(f"Service: {service.uuid} ({service.description})")
                    for char in service.characteristics:
                        props = ",".join(char.properties)
                        print(f"  └─ Char: {char.uuid} | Props: [{props}] | Desc: {char.description}")
                print("=" * 60)
                print("👋 断开连接测试...")
    except Exception as e:
        print(f"🔥 连接或读取失败: {e}")

async def main():
    # 1. 扫描
    devices = await scan_devices()
    if not devices:
        return

    # 2. 交互选择
    target_mac = input("\n请输入你要连接的设备 MAC 地址 (直接回车退出): ").strip()
    
    if target_mac:
        # 3. 深入检测
        await inspect_device(target_mac)
    else:
        print("Bye!")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"运行时错误: {e}")