import serial
import time
from datetime import datetime
import os
import argparse

class AUVDataLogger:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200):
        """Initialize the data logger"""
        # ตรวจสอบและปิด Serial port ที่อาจค้างอยู่
        try:
            temp_serial = serial.Serial(port)
            temp_serial.close()
        except:
            pass
            
        time.sleep(1)  # รอให้พอร์ตถูกปิดสมบูรณ์
        
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=1,
                write_timeout=1,
                rtscts=True,  # เปิดใช้ hardware flow control
                dsrdtr=True   # เปิดใช้ hardware flow control
            )
            
            # Reset the Arduino/ESP32
            self.serial_port.setDTR(False)
            time.sleep(0.1)
            self.serial_port.setDTR(True)
            
            # รอให้ ESP32 เริ่มต้นใหม่และพร้อมส่งข้อมูล
            time.sleep(2)
            
            # Flush buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise
            
        self.columns = [
            'timestamp',
            'angleY', 'angleZ', 'depth',
            'kP_Y', 'kI_Y', 'kD_Y',
            'kP_Z', 'kI_Z', 'kD_Z',
            'kP_D', 'kI_D', 'kD_D',
            'targetDepth', 'device_timestamp'
        ]
        
        # สร้างชื่อไฟล์ด้วย timestamp
        self.filename = f'AUV_{datetime.now().strftime("%Y%m%d_%H%M%S")}.txt'
        
        # เขียนส่วนหัว
        with open(self.filename, 'w', buffering=1) as f:
            header = ','.join(self.columns)
            f.write(header + '\n')
        
        self.data_count = 0
        self.running = True
        
    def wait_for_data(self, timeout=10):
        """รอจนกว่าจะได้รับข้อมูลจาก ESP32"""
        print("Waiting for data from ESP32...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                if line.startswith('DATA'):
                    print("Successfully receiving data from ESP32")
                    return True
            time.sleep(0.1)
        return False

    def start_logging(self):
        """Start logging data from serial port"""
        print(f"Starting data logging to {self.filename}")
        print("Press Ctrl+C to stop logging")
        
        # ตรวจสอบว่าได้รับข้อมูลจริงๆ
        if not self.wait_for_data():
            print("No data received from ESP32. Please check the connection and try again.")
            self.cleanup()
            return
        
        with open(self.filename, 'a', buffering=1) as data_file:
            try:
                while self.running:
                    try:
                        if self.serial_port.in_waiting:
                            line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                            
                            if line.startswith('DATA'):
                                try:
                                    values = line.split(',')[1:]
                                    
                                    if len(values) == 14:
                                        current_time = datetime.now().timestamp()
                                        data_line = f"{current_time}," + ','.join(values) + '\n'
                                        data_file.write(data_line)
                                        data_file.flush()
                                        
                                        self.data_count += 1
                                        
                                        print(f"\rRecords: {self.data_count} | Latest: angleY={values[0]}°, "
                                              f"angleZ={values[1]}°, depth={values[2]}m", end='')
                                        
                                except (ValueError, IndexError) as e:
                                    print(f"\nError parsing data: {e} - Line: {line}")
                    except serial.SerialException as e:
                        print(f"\nSerial port error: {e}")
                        break
                        
                    time.sleep(0.01)
                    
            except KeyboardInterrupt:
                print(f"\nLogging stopped by user")
                print(f"Total records saved: {self.data_count}")
            finally:
                self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        print(f"\nData saved to {self.filename}")

def main():
    parser = argparse.ArgumentParser(description='AUV Data Logger')
    parser.add_argument('--port', default='/dev/ttyUSB0', 
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, 
                        help='Baud rate (default: 115200)')
    args = parser.parse_args()
    
    try:
        logger = AUVDataLogger(port=args.port, baud_rate=args.baud)
        logger.start_logging()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()