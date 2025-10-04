#!/usr/bin/env python3
import time
from datetime import datetime

def display_current_time():
    print("Программа вывода текущего времени")
    print("Для выхода нажмите Ctrl+C")
    print("-" * 30)
    
    try:
        while True:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"\rТекущее время: {current_time}", end="", flush=True)
            time.sleep(5)
    except KeyboardInterrupt:
        print("\n\nПрограмма завершена. До свидания!")

if __name__ == "__main__":
    display_current_time()
