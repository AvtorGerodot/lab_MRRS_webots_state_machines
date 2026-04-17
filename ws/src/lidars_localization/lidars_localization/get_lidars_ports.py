#!/usr/bin/env python3

import serial
import time
import glob
import sys


def get_hokuyo_id(port):
    """Получить серийный номер лидара по порту"""
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.1)
        
        ser.flushInput()
        ser.flushOutput()
        
        ser.write(b'VV\n')
        time.sleep(0.2)
        
        response = ser.read(500).decode('ascii', errors='ignore')
        ser.close()
        
        # Ищем строку с серийным номером
        lines = response.split('\n')
        for line in lines:
            if 'SERI' in line:
                serial_num = line.split(':')[-1].strip().rstrip(';]')
                return serial_num
        
        return None
        
    except Exception:
        return None


def find_lidar_ports():
    """Найти порты для конкретных лидаров"""
    # Ищем все ttyACM порты
    tty_ports = glob.glob('/dev/ttyACM*')
    
    # Переменные для хранения портов
    front_port = None  # для H1766499
    rear_port = None   # для H1766497
    
    for port in sorted(tty_ports):
        serial_id = get_hokuyo_id(port)
        
        if serial_id:
            # H1766497 - передний, H1766499 - задний (как вы сказали)
            if 'H1766497' in serial_id:
                front_port = port
            elif 'H1766499' in serial_id:
                rear_port = port
    
    return front_port, rear_port


def main():
    # По умолчанию возвращаем оба порта через пробел
    front_port, rear_port = find_lidar_ports()
    
    # Возвращаем результат: "front_port rear_port" или "NONE NONE" если не найдены
    front_result = front_port if front_port else "NONE"
    rear_result = rear_port if rear_port else "NONE"
    
    print(f"{front_result} {rear_result}")
    
    # Для отладки в stderr
    print(f"Front lidar (H1766497): {front_port or 'NOT FOUND'}", file=sys.stderr)
    print(f"Rear lidar (H1766499): {rear_port or 'NOT FOUND'}", file=sys.stderr)


if __name__ == '__main__':
    main()
