#!/usr/bin/env python3
import argparse
import asyncio
import json
from dataclasses import dataclass
from typing import Optional

from bleak import BleakClient


VENDOR_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
VENDOR_NOTIFY_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
VENDOR_WRITE_PRIMARY_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"
VENDOR_WRITE_SECONDARY_UUID = "0000fff3-0000-1000-8000-00805f9b34fb"


@dataclass
class CharacteristicSelection:
    service_uuid: str
    characteristic_uuid: str
    properties: list[str]


def normalize_uuid(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    trimmed = value.strip()
    if not trimmed:
        return None
    return trimmed.lower()


def characteristic_is_writable(properties: list[str]) -> bool:
    return "write" in properties or "write-without-response" in properties


def characteristic_is_notifiable(properties: list[str]) -> bool:
    return "notify" in properties or "indicate" in properties


def select_write_characteristic(services, requested_service_uuid: Optional[str], requested_char_uuid: Optional[str]) -> CharacteristicSelection:
    requested_service_uuid = normalize_uuid(requested_service_uuid)
    requested_char_uuid = normalize_uuid(requested_char_uuid)

    if requested_service_uuid and requested_char_uuid:
        for service in services:
            if service.uuid.lower() != requested_service_uuid:
                continue
            for char in service.characteristics:
                if char.uuid.lower() == requested_char_uuid:
                    return CharacteristicSelection(service.uuid, char.uuid, list(char.properties))
        raise RuntimeError("未找到指定的可写特征值")

    candidates: list[tuple[int, int, CharacteristicSelection]] = []
    for service_index, service in enumerate(services):
        for char_index, char in enumerate(service.characteristics):
            properties = list(char.properties)
            if not characteristic_is_writable(properties):
                continue
            service_uuid = service.uuid.lower()
            char_uuid = char.uuid.lower()
            if service_uuid == VENDOR_SERVICE_UUID and char_uuid == VENDOR_WRITE_PRIMARY_UUID:
                priority = 0
            elif service_uuid == VENDOR_SERVICE_UUID and char_uuid == VENDOR_WRITE_SECONDARY_UUID:
                priority = 1
            elif service_uuid == VENDOR_SERVICE_UUID:
                priority = 2
            else:
                priority = 10
            candidates.append(
                (
                    priority,
                    service_index * 100 + char_index,
                    CharacteristicSelection(service.uuid, char.uuid, properties),
                )
            )

    if not candidates:
        raise RuntimeError("未找到可写特征值")

    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2]


def select_notify_characteristic(services, requested_notify_uuid: Optional[str]) -> Optional[CharacteristicSelection]:
    requested_notify_uuid = normalize_uuid(requested_notify_uuid)
    candidates: list[tuple[int, int, CharacteristicSelection]] = []

    for service_index, service in enumerate(services):
        for char_index, char in enumerate(service.characteristics):
            properties = list(char.properties)
            if not characteristic_is_notifiable(properties):
                continue
            char_uuid = char.uuid.lower()
            if requested_notify_uuid and char_uuid != requested_notify_uuid:
                continue
            if char_uuid == VENDOR_NOTIFY_UUID:
                priority = 0
            else:
                priority = 10
            candidates.append(
                (
                    priority,
                    service_index * 100 + char_index,
                    CharacteristicSelection(service.uuid, char.uuid, properties),
                )
            )

    if not candidates:
        return None

    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2]


async def execute_request(payload: dict) -> dict:
    mac = payload["mac"]
    service_uuid = normalize_uuid(payload.get("service_uuid"))
    write_char_uuid = normalize_uuid(payload.get("write_char_uuid"))
    notify_char_uuid = normalize_uuid(payload.get("notify_char_uuid"))
    command_hex = payload.get("command_hex")
    expect_notification = bool(payload.get("expect_notification"))
    notification_timeout_ms = int(payload.get("notification_timeout_ms", 5000))

    async with BleakClient(mac, timeout=10.0) as client:
        services = client.services
        write_char = select_write_characteristic(services, service_uuid, write_char_uuid)
        notify_char = select_notify_characteristic(services, notify_char_uuid) if expect_notification else None

        notification_hex = None
        notify_future = None

        if expect_notification:
            if notify_char is None:
                raise RuntimeError("未找到通知特征值")
            loop = asyncio.get_running_loop()
            notify_future = loop.create_future()

            def handle_notification(_, data: bytearray):
                if not notify_future.done():
                    notify_future.set_result(bytes(data).hex().upper())

            await client.start_notify(notify_char.characteristic_uuid, handle_notification)

        try:
            if command_hex:
                data = bytes.fromhex(command_hex)
                write_with_response = "write" in write_char.properties and "write-without-response" not in write_char.properties
                await client.write_gatt_char(write_char.characteristic_uuid, data, response=write_with_response)

            if expect_notification:
                notification_hex = await asyncio.wait_for(
                    notify_future, timeout=notification_timeout_ms / 1000
                )
        finally:
            if expect_notification and notify_char is not None:
                try:
                    await client.stop_notify(notify_char.characteristic_uuid)
                except Exception:
                    pass

        if command_hex:
            message = f"已通过 Bleak fallback 连接并发送指令: {command_hex}"
        else:
            message = "已通过 Bleak fallback 建立连接"

        return {
            "success": True,
            "message": message,
            "service_uuid": write_char.service_uuid,
            "write_char_uuid": write_char.characteristic_uuid,
            "notify_char_uuid": notify_char.characteristic_uuid if notify_char else None,
            "notification_hex": notification_hex,
        }


async def async_main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", required=True)
    args = parser.parse_args()

    try:
        payload = json.loads(args.request)
        response = await execute_request(payload)
    except Exception as exc:
        response = {
            "success": False,
            "message": str(exc),
            "service_uuid": None,
            "write_char_uuid": None,
            "notify_char_uuid": None,
            "notification_hex": None,
        }

    print(json.dumps(response, ensure_ascii=False))
    return 0


def main() -> int:
    return asyncio.run(async_main())


if __name__ == "__main__":
    raise SystemExit(main())
