#VER1005 - 1.50 (2025-2030)
# HIHIHIHIHI
# -------------------------------------------
# IMPORTS
# -------------------------------------------

import urequests
import machine
import random
import utime
import time
import re
import ujson
import math
from machine import UART, Pin, I2C, ADC
import config
from config import MQTT_BROKER, MQTT_PORT, MQTT_TOPIC, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, LTE_APN
import uos
import gc
from machine import PWM

# -------------------------------------------
# INITIALIZE HARDWARE
# -------------------------------------------

# UART for K96 Sensor (Grove Port 1 - GP0, GP1)
uart_k96 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=500)

# K96 Sensor and Heatpad Power (GP8 = Positive, GP9 = Negative)
k96_heatpad_power_pos = Pin(8, Pin.OUT)
k96_heatpad_power_neg = Pin(9, Pin.OUT)

# Sample Pump Power (GP10 = Positive, GP11 = Negative)
pump_power_pos = Pin(10, Pin.OUT)
pump_power_neg = Pin(11, Pin.OUT)

# LTE Module UART (Use UART 1 for LTE communication)
uart_lte = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5), timeout=500)

# Pin Definitions
lte_pwrkey = Pin(16, Pin.OUT)

SAMPLE_PUMP_DURATION = 50  # Duration (in seconds) to run the sample pump (used in run_sample_pump)
HEATPAD_STABILIZATION_TIME = 45  # Duration (in seconds) to wait for heatpad stabilization (used in heatpad_control)

OTA_URL = "https://raw.githubusercontent.com/LukeAlator/Verion-Test/main/latest.py"

# -------------------------------------------
# FUNCTIONS
# -------------------------------------------

LOG_FILE = "log.txt"
LAST_RESET_FILE = "last_reset.txt"

def log(message, level="INFO"):
    """Simple logging function that writes to a log file."""
    timestamp = utime.localtime()
    formatted_time = f"{timestamp[0]:04d}-{timestamp[1]:02d}-{timestamp[2]:02d} {timestamp[3]:02d}:{timestamp[4]:02d}:{timestamp[5]:02d}"
    log_message = f"[{formatted_time}] [{level}] {message}"
    print(log_message)
    
    # Write the log message to a file
    try:
        with open(LOG_FILE, "a") as log_file:
            log_file.write(log_message + "\n")
    except Exception as e:
        print(f"Failed to write log to file: {e}")

def truncate_log_file():
    """Deletes the log file if a day has passed since the last reset."""
    try:
        current_time = utime.time()
        reset_needed = False

        # Check if the last reset time file exists
        try:
            with open(LAST_RESET_FILE, "r") as f:
                last_reset_time = float(f.read().strip())
                # Check if a day has passed since the last reset
                reset_needed = current_time - last_reset_time >= 86400  # 86400 seconds in a day
        except OSError:
            # If the file does not exist, assume a reset is needed
            reset_needed = True

        if reset_needed:
            # Delete the log file if it exists
            try:
                uos.stat(LOG_FILE)  # Check if the log file exists
                uos.remove(LOG_FILE)
                print("Log file deleted to start a new one for the day.")
            except OSError:
                print("Log file does not exist or could not be deleted.")

            # Update the last reset time
            with open(LAST_RESET_FILE, "w") as f:
                f.write(str(current_time))
                print(f"Last reset time updated: {current_time}")
    except Exception as e:
        print(f"Failed to reset log file: {e}")

def send_at_command(command, delay=1):
    """Sends an AT command and reads the response."""
    log(f"Sending: {command}")
    uart_lte.write((command + "\r\n").encode("utf-8"))
    utime.sleep(delay)
    response = uart_lte.read()
    if response:
        try:
            # Decode the response and replace invalid characters manually
            decoded_response = response.decode("utf-8").strip()
            log(f"Response: {decoded_response}")
            return decoded_response
        except UnicodeError as e:
            # Replace invalid characters manually if decoding fails
            decoded_response = "".join(chr(c) if c < 128 else "?" for c in response)
            log(f"UnicodeError while decoding response: {e}. Recovered response: {decoded_response}", level="ERROR")
            return decoded_response
    return ""

def control_lte_power(state, reset=False):
    """Controls the LTE module power state: ON, OFF, wake up, or reset."""
    if state:
        if reset:
            log("Resetting LTE modem...", level="INFO")
        else:
            log("Powering on LTE modem...", level="INFO")
        lte_pwrkey.value(1)
        time.sleep(2)
        lte_pwrkey.value(0)
        time.sleep(5)  # Give some time for the modem to wake up or reset
        if reset:
            log("LTE modem reset complete.", level="INFO")
        else:
            log("LTE modem power-on complete.", level="INFO")
    else:
        log("Powering off LTE modem...", level="INFO")
        lte_pwrkey.value(0)
        time.sleep(2)
        lte_pwrkey.value(1)
        time.sleep(2)
        log("LTE modem power-off complete.", level="INFO")

def setup_lte(initial_setup=False, retries=3):
    """Initialize LTE module, set APN, and activate PDP context."""
    for attempt in range(retries):
        print("Powering on LTE module...")
        control_lte_power(state=True)
        time.sleep(5)  # Give some time for the module to power on

        response = send_at_command("AT", 2)
        if "OK" not in response:
            print("❌ No response to AT command. Check LTE module power and connections.")
            if attempt < retries - 1:
                log(f"Retrying LTE setup... ({attempt + 1}/{retries})", level="WARNING")
                continue
            return False

        if initial_setup:
            print("Setting LTE functionality...")
            response = send_at_command("AT+CFUN=1", 5)
            print(f"AT+CFUN=1 Response: {response}")
            response = send_at_command("AT+CSQ", 2)
            print(f"AT+CSQ Response: {response}")
            response = send_at_command("AT+CGDCONT?", 2)
            print(f"AT+CGDCONT? Response: {response}")
            time.sleep(5)

            print("🌐 Configuring PDP context...")
            response = send_at_command("AT+CGACT=0,1", 3)
            print(f"AT+CGACT=0,1 Response: {response}")
            if "OK" not in response:
                print("❌ Failed to deactivate PDP context! Response:", response)
                if attempt < retries - 1:
                    log(f"Retrying LTE setup... ({attempt + 1}/{retries})", level="WARNING")
                    continue
                return False

            response = send_at_command(f'AT+CGDCONT=1,"IP","{LTE_APN}"', 3)
            print(f'AT+CGDCONT=1,"IP","{LTE_APN}" Response: {response}')
            if "OK" not in response:
                print("❌ Failed to set APN! Response:", response)
                if attempt < retries - 1:
                    log(f"Retrying LTE setup... ({attempt + 1}/{retries})", level="WARNING")
                    continue
                return False

            response = send_at_command("AT+CGACT=1,1", 3)
            print(f"AT+CGACT=1,1 Response: {response}")
            if "OK" not in response:
                print("❌ Failed to activate PDP context! Response:", response)
                if attempt < retries - 1:
                    log(f"Retrying LTE setup... ({attempt + 1}/{retries})", level="WARNING")
                    continue
                return False

        response = send_at_command("AT+CGACT?", 2)
        print(f"AT+CGACT? Response: {response}")
        set_sms_text_mode()  # Set SMS mode to Text mode once during initial setup

        # Ensure LTE connection is established before syncing time
        if not check_lte_connection():
            print("❌ LTE connection not established.")
            if attempt < retries - 1:
                log(f"Retrying LTE setup... ({attempt + 1}/{retries})", level="WARNING")
                continue
            return False

        print("✅ LTE setup complete.")
        return True

    return False

def check_lte_connection():
    """Check if the LTE connection is established."""
    response = send_at_command("AT+CGATT?", 2)
    if "+CGATT: 1" in response:
        print("✅ LTE connection established.")
        return True
    else:
        print("❌ LTE connection not established.")
        return False

def set_sms_text_mode():
    """Sets the SMS mode to Text mode."""
    print("Setting SMS mode to Text mode...")
    response = send_at_command("AT+CMGF=1", 5)
    if "OK" in response:
        print("SMS mode set to Text mode successfully.")
    else:
        print("Failed to set SMS mode to Text mode. Response:", response)

def run_sample_pump(duty_cycle=32768):
    """Turns ON the sample pump for a specified duration at the given duty cycle, then turns it OFF."""
    print('Sample pump powered ON.')
    pwm = PWM(pump_power_pos)
    pwm.freq(1000)  # Set frequency to 1 kHz
    pwm.duty_u16(duty_cycle)  # Set duty cycle (50% duty cycle is 32768 for 16-bit resolution)
    pump_power_neg.value(0)
    time.sleep(SAMPLE_PUMP_DURATION)  # Use the constant here
    pwm.deinit()  # Turn off PWM
    pump_power_pos.init(Pin.OUT)  # Reinitialize the pin as a regular GPIO pin
    pump_power_pos.value(0)  # Explicitly set the pump power pin to low
    pump_power_neg.value(0)  # Ensure the negative pin is also set to low
    print('Sample pump powered OFF.')

def heatpad_control():
    """Controls the heat pad until temperature is ≥15°C or 200 seconds timeout, then waits for stable K96 readings."""
    print("🔥 Turning on heat pad and K96 sensor...")
    enable_k96_heatpad_power(True)

    start_time = time.time()
    while time.time() - start_time < 200:  # Max wait: 200 seconds
        k96_data = read_k96(print_readings=False)  # Read K96 data without printing
        temp = k96_data.get("Temperature", 0)  # Extract the temperature value
        if temp and temp >= 15:
            print(f"✅ Temperature reached 15°C (Temp: {temp}°C)")
            break
        time.sleep(5)

    print(f"Waiting {HEATPAD_STABILIZATION_TIME} seconds for stable K96 readings...")
    time.sleep(HEATPAD_STABILIZATION_TIME)  # Use the constant here

def enable_k96_heatpad_power(enable):
    """Turns K96 sensor and heatpad power ON or OFF using GPIO pins."""
    k96_heatpad_power_pos.value(1 if enable else 0)
    k96_heatpad_power_neg.value(0)
    print(f"📟 K96 sensor and heatpad {'ON' if enable else 'OFF'}.")
    

def get_signal_strength():
    """Returns the signal strength (RSSI) of the LTE connection."""
    response = send_at_command("AT+CSQ")
    match = re.search(r'\+CSQ: (\d+),', response)
    if match:
        rssi = int(match.group(1))
        if rssi == 99:
            return None  # Signal strength not detectable
        return rssi
    return None

def read_k96(print_readings=True):
    """Reads data from the K96 sensor by sending commands and parsing responses."""
    sensor_data = {}
    for i, command in enumerate(config.K96_COMMANDS):
        response = send_command(command)
        value = parse_response(response, command, config.K96_UNITS[i], config.K96_OFFSETS[i], config.K96_SLOPES[i])
        sensor_data[config.K96_NAMES[i]] = value if value is not None else 0.0
        if print_readings:
            print(f"{config.K96_NAMES[i]}: {value}")
    return sensor_data

def send_command(command):
    """Sends a command to the K96 sensor via UART and reads the response."""
    try:
        uart_k96.write(bytearray.fromhex(command))
        time.sleep(1)
        response = uart_k96.read()
        if response:
            return response
        else:
            raise ValueError("No response received from K96 sensor")
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

def parse_response(response, command, unit, offset, slope):
    """Parses the response from the K96 sensor and applies unit conversion, offset, and slope."""
    if response and len(response) >= 7:
        size = int(command[12:14], 16)
        if len(response) < 3 + size:
            return None
        value = int.from_bytes(response[3:3 + size], "big", True)
        return round(value * unit * slope + offset, 2)  # Apply slope here
    return None

def check_sms():
    """Checks for SMS and triggers actions based on the content."""
    log("Checking for unread SMS messages...", level="INFO")
    response = send_at_command('AT+CMGL="REC UNREAD"')
    log(f"Raw SMS response: {response}", level="DEBUG")
    
    if response and "+CMGL" in response:
        log("Unread SMS detected. Processing...", level="INFO")
        lines = response.split("\n")
        for i, line in enumerate(lines):
            log(f"Processing SMS line: {line}", level="DEBUG")
            
            # Check if the line contains the SMS metadata
            if "+CMGL" in line:
                # Extract the SMS content from the next non-empty line
                for j in range(i + 1, len(lines)):
                    sms_content = lines[j].strip()
                    if sms_content:  # Skip empty lines
                        log(f"Extracted SMS content: {sms_content}", level="DEBUG")
                        
                        # Check for sample interval update
                        if "SAMPLE_INTERVAL=" in sms_content:
                            try:
                                new_interval = int(sms_content.split("=")[1])
                                config.SAMPLE_INTERVAL = new_interval
                                log(f"Updated sample interval to {new_interval} seconds", level="INFO")
                            except ValueError:
                                log("⚠️ Error parsing sample interval from SMS.", level="ERROR")
                        
                        # Check for OTA update trigger
                        elif "update" in sms_content.lower():
                            log("Update SMS received. Initiating OTA update...", level="INFO")
                            perform_ota_update()
                        
                        # Stop processing after finding the SMS content
                        break
    else:
        log("No unread SMS messages found.", level="INFO")
                    
def perform_ota_update():
    """Performs an OTA update by downloading and replacing the current script."""
    try:
        log("Starting OTA update process...", level="INFO")
        log(f"Downloading firmware from {OTA_URL}...", level="INFO")
        
        response = urequests.get(OTA_URL)
        log(f"HTTP status code: {response.status_code}", level="DEBUG")
        
        if response.status_code == 200:
            new_code = response.text
            log("Firmware downloaded successfully. Writing to file...", level="INFO")
            
            # Write the new code to the current script file
            with open("main.py", "w") as script_file:  # Replace "main.py" with your script's filename
                script_file.write(new_code)
            
            log("Firmware updated successfully. Rebooting...", level="INFO")
            reboot_system()
        else:
            log(f"Failed to download firmware. HTTP status code: {response.status_code}", level="ERROR")
    except Exception as e:
        log(f"Error during OTA update: {e}", level="ERROR")
                    
def enable_sleep_mode():
    """Enables sleep mode using AT+CSCLK command."""
    print("Enabling sleep mode for LTE module...")
    response = send_at_command("AT+CSCLK=1", 5)
    if "OK" in response:
        print("Sleep mode enabled successfully.")
    else:
        print("Failed to enable sleep mode. Response:", response)

def resend_failed_packets(client):
    """Retry sending failed packets stored in a local file."""
    try:
        with open("failed_packets.json", "r") as f:
            lines = f.readlines()
        remaining_packets = []
        for line in lines:
            try:
                packet_with_timestamp = ujson.loads(line.strip())
                timestamp = packet_with_timestamp["timestamp"]
                packet = packet_with_timestamp["data"]
                print("Sending failed packet...")
                if not send_data_to_mqtt(client, packet, timestamp):
                    remaining_packets.append(line)
            except Exception as e:
                print(f"Error processing failed packet: {e}, Line: {line.strip()}")
                remaining_packets.append(line)
        with open("failed_packets.json", "w") as f:
            for packet in remaining_packets:
                f.write(packet + "\n")
    except OSError:
        print("No failed packets found.")

def save_failed_packet(packet, timestamp):
    """Saves the failed packet to a file for retry."""
    packet_with_timestamp = {"timestamp": timestamp, "data": packet}
    try:
        with open("failed_packets.json", "a") as file:
            ujson.dump(packet_with_timestamp, file)
            file.write("\n")
    except Exception as e:
        print(f"Error saving failed packet: {e}")

def flatten_dict(d, parent_key='', sep='_'):
    """Flattens a nested dictionary for easier handling."""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def send_data_to_mqtt(client, data, timestamp):
    """Sends sensor data to an MQTT broker in the required JSON format."""
    flattened_data = flatten_dict(data)
    print(f"📦 Formatted Data: {flattened_data}")

    # Add timestamp to each variable
    payload = {key: {"value": value, "timestamp": timestamp} for key, value in flattened_data.items()}

    # Publish all data in a single JSON payload
    payload_str = ujson.dumps(payload)
    if not client.publish(MQTT_TOPIC, payload_str):
        print(f"Failed to publish: {payload_str}")
        return False
    print(f"Published: {payload_str}")

    return True

def send_data_with_retry(client, sensor_data, wakeup_timestamp):
    """Attempts to send data with a retry mechanism."""
    packet_saved = False  # Flag to track if the packet has been saved

    for attempt in range(config.PACKET_RETRIES):
        try:
            if client.connect():
                resend_failed_packets(client)  # Attempt to resend any failed packets
                log("Sending current values...", level="INFO")
                
                # Trigger garbage collection to free up memory
                gc.collect()
                
                if send_data_to_mqtt(client, sensor_data, wakeup_timestamp):
                    client.disconnect()
                    
                    # Check for SMS after sending data
                    response = send_at_command('AT+CMGL="REC UNREAD"')
                    if response and "+CMGL" in response:
                        log("SMS received. Processing...", level="INFO")
                        check_sms()
                    else:
                        log("No SMS received.", level="INFO")
                    
                    enable_sleep_mode()  # Enable sleep mode after disconnecting
                    return True
                else:
                    if not packet_saved:
                        save_failed_packet(sensor_data, wakeup_timestamp)
                        packet_saved = True
                    client.disconnect()
            else:
                if not packet_saved:
                    save_failed_packet(sensor_data, wakeup_timestamp)
                    packet_saved = True
            
            # Wait for a random short interval before retrying
            retry_interval = random.uniform(5, 15)  # Random wait time between 5 and 15 seconds
            log(f"Retrying in {retry_interval:.2f} seconds...", level="WARNING")
            time.sleep(retry_interval)
        
        except MemoryError as e:
            log(f"Memory allocation failed: {e}", level="ERROR")
            gc.collect()  # Trigger garbage collection to free up memory
            if not packet_saved:
                save_failed_packet(sensor_data, wakeup_timestamp)
                packet_saved = True
            # Wait for a random short interval before retrying
            retry_interval = random.uniform(5, 15)  # Random wait time between 5 and 15 seconds
            log(f"Retrying in {retry_interval:.2f} seconds after memory error...", level="WARNING")
            time.sleep(retry_interval)
    
    # If all retries fail, check LTE connection and reset the modem if needed
    log("All retries failed. Checking LTE connection...", level="ERROR")
    if not check_lte_connection():
        log("LTE connection lost. Resetting modem and reinitializing...", level="ERROR")
        control_lte_power(state=True, reset=True)
        if not setup_lte():
            log("Failed to setup LTE after modem reset. Exiting...", level="ERROR")
            return False
    
    # Attempt to send the packet again once after resetting the modem
    if client.connect():
        resend_failed_packets(client)  # Attempt to resend any failed packets
        log("Retrying to send current values after modem reset...", level="INFO")
        
        # Trigger garbage collection to free up memory
        gc.collect()
        
        if send_data_to_mqtt(client, sensor_data, wakeup_timestamp):
            client.disconnect()
            
            # Check for SMS after sending data
            response = send_at_command('AT+CMGL="REC UNREAD"')
            if response and "+CMGL" in response:
                log("SMS received. Processing...", level="INFO")
                check_sms()
            else:
                log("No SMS received.", level="INFO")
            
            enable_sleep_mode()  # Enable sleep mode after disconnecting
            return True
        else:
            if not packet_saved:
                save_failed_packet(sensor_data, wakeup_timestamp)
                packet_saved = True
            client.disconnect()
    
    return False

class CustomMQTTClient:
    def __init__(self, client_id, broker, topic, port, user, password):
        self.client_id = client_id
        self.broker = broker
        self.topic = topic
        self.port = port
        self.user = user
        self.password = password

    def connect(self):
        """Connects to the MQTT broker."""
        try:
            # Step 3: Start MQTT service
            response = send_at_command("AT+CMQTTSTART", 5)
            if "OK" not in response:
                print("❌ Failed to start MQTT service! Response:", response)
                return False
            
            # Step 4: Acquire a client
            response = send_at_command(f'AT+CMQTTACCQ=0,"{self.client_id}"', 5)
            if "OK" not in response:
                print("❌ Failed to acquire MQTT client! Response:", response)
                return False
            
            # Step 7: Connect to MQTT server
            connect_command = f'AT+CMQTTCONNECT=0,"tcp://{self.broker}:{self.port}",60,1,"{self.user}","{self.password}"'
            print(f"📡 Sending: {connect_command}")
            response = send_at_command(connect_command, 5)
            if "OK" not in response:
                print("❌ Failed to connect to MQTT server! Response:", response)
                return False
            
            return True
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
            return False

    def publish(self, topic, message):
        """Publishes a message to the MQTT broker."""
        try:
            # Step 10: Input the topic of a publish message
            response = send_at_command(f'AT+CMQTTTOPIC=0,{len(topic)}', 5)
            if ">" in response:
                uart_lte.write((topic + "\r\n").encode("utf-8"))
                utime.sleep(1)
            else:
                print("❌ Failed to input topic! Response:", response)
                return False
            
            # Step 11: Input the payload of a publish message
            response = send_at_command(f'AT+CMQTTPAYLOAD=0,{len(message)}', 5)
            if ">" in response:
                uart_lte.write((message + "\r\n").encode("utf-8"))
                utime.sleep(1)
            else:
                print("❌ Failed to input payload! Response:", response)
                return False
            
            # Step 12: Publish message
            response = send_at_command("AT+CMQTTPUB=0,1,60", 5)
            if "OK" not in response:
                print("❌ Failed to publish message! Response:", response)
                return False
            
            return True
        except Exception as e:
            print(f"Error publishing to MQTT broker: {e}")
            return False

    def disconnect(self):
        """Disconnects from the MQTT broker."""
        try:
            # Step 13: Disconnect from the server
            response = send_at_command("AT+CMQTTDISC=0,120", 5)
            if "OK" not in response:
                print("❌ Failed to disconnect from MQTT server! Response:", response)
            
            # Step 14: Release the client
            response = send_at_command("AT+CMQTTREL=0", 5)
            if "OK" not in response:
                print("❌ Failed to release MQTT client! Response:", response)
            
            # Step 15: Stop MQTT service
            response = send_at_command("AT+CMQTTSTOP", 5)
            if "OK" not in response:
                print("❌ Failed to stop MQTT service! Response:", response)
        except Exception as e:
            print(f"Error disconnecting from MQTT broker: {e}")
            
def get_network_time(retries=3):
    """Synchronizes the microcontroller's RTC and the modem's RTC with the network time obtained from the LTE module."""
    for attempt in range(retries):
        send_at_command("AT+CNTP=\"pool.ntp.org\",0")  # Set NTP server
        time.sleep(1)
        send_at_command("AT+CNTP")  # Sync time
        time.sleep(1)
        
        response = send_at_command("AT+CCLK?")  # Get time
        log(f"Raw modem time response: {response}")
        
        # Extract the time string from the response
        match = re.search(r'\+CCLK: "(.+)"', response)
        if match:
            time_str = match.group(1)
            # Parse the time string
            year = int("20" + time_str[0:2])
            month = int(time_str[3:5])
            day = int(time_str[6:8])
            hour = int(time_str[9:11])
            minute = int(time_str[12:14])
            second = int(time_str[15:17])
            
            # Check if the year is within the valid range
            if 2025 <= year <= 2030:
                # Set MicroPython RTC
                rtc = machine.RTC()
                rtc.datetime((year, month, day, 0, hour, minute, second, 0))
                log(f"Updated RTC to: {year}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}")
                
                # Set the modem RTC
                modem_time_str = f'{year % 100:02d}/{month:02d}/{day:02d},{hour:02d}:{minute:02d}:{second:02d}+00'
                response = send_at_command(f'AT+CCLK="{modem_time_str}"')
                log(f"Set modem RTC response: {response}")
                
                return True
            else:
                log("Invalid year received from modem time. Retrying...", level="ERROR")
        else:
            log("Failed to parse modem time. Retrying...", level="ERROR")
        
        log(f"Retrying NTP synchronization... ({attempt + 1}/{retries})")
        time.sleep(2)
    
    log("Failed to synchronize time after multiple attempts. Rebooting...", level="ERROR")
    reboot_system()
    return False

def reboot_system():
    """Reboots the system to apply the new firmware."""
    log("Rebooting system to apply new firmware...", level="INFO")
    time.sleep(5)  # Wait for a few seconds before rebooting
    machine.reset()  # Reboot the system

def send_data_to_mqtt(client, data, timestamp):
    """Sends sensor data to an MQTT broker in the required JSON format."""
    flattened_data = flatten_dict(data)
    print(f"📦 Formatted Data: {flattened_data}")

    # Add timestamp to each variable
    payload = {key: {"value": value, "timestamp": timestamp} for key, value in flattened_data.items()}

    # Publish all data in a single JSON payload
    payload_str = ujson.dumps(payload)
    if not client.publish(MQTT_TOPIC, payload_str):
        print(f"Failed to publish: {payload_str}")
        return False
    print(f"Published: {payload_str}")

    return True

def collect_sensor_data():
    """Collects data from various sensors and returns it as a dictionary."""
    sensor_data = {
        "Device_Type": config.DEVICE_TYPE,
        "K96": read_k96(),
        "Signal_Strength": get_signal_strength(),
        "Firmware": config.FIRMWARE_VERSION
    }
    return sensor_data

# -------------------------------------------
# MAIN PROGRAM LOOP
# -------------------------------------------

try:
    base_interval = config.SAMPLE_INTERVAL
    log("Taking initial measurement on power-up...")
    time.sleep(2)
    # Power on LTE modem and set it up
    if not setup_lte(initial_setup=True):
        log("Failed to setup LTE after multiple attempts. Rebooting...", level="ERROR")
        reboot_system()  # Reboot the system if LTE setup fails

    # Get network time and set it on the microcontroller
    if not get_network_time():
        raise SystemExit  # Exit if time synchronization fails and system is rebooting

    # Capture the timestamp when the unit wakes up
    wakeup_timestamp = int(utime.time() * 1000)
    
    # Truncate the log file at the start of each sample interval
    truncate_log_file()
    
    run_sample_pump(duty_cycle=32768)  # Set the duty cycle to 50%

    heatpad_control()
    
    sensor_data = collect_sensor_data()
    enable_k96_heatpad_power(False)

    # Send the data using the LTE modem with retry mechanism
    client = CustomMQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, MQTT_TOPIC, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
    if not send_data_with_retry(client, sensor_data, wakeup_timestamp):
        log("Failed to send data after multiple attempts.", level="ERROR")

    control_lte_power(state=False)
    log("Initial setup complete. Entering continuous monitoring loop...")

    # Continuous monitoring loop
    while True:
        try:
            next_sample_time = math.ceil(utime.time() / config.SAMPLE_INTERVAL) * config.SAMPLE_INTERVAL
            sleep_duration = next_sample_time - utime.time()
            log(f"Sleeping for {sleep_duration} seconds until next sample time...", level="INFO")
            utime.sleep(sleep_duration)

            log(f"[{utime.localtime()}] Taking scheduled measurement...", level="INFO")

            # Reset the log file daily
            truncate_log_file()

            # Capture the timestamp when the unit wakes up
            wakeup_timestamp = int(utime.time() * 1000)

            run_sample_pump(duty_cycle=32768)  # Set the duty cycle to 50%

            heatpad_control()

            # Power on LTE modem to send data
            control_lte_power(state=True)
            time.sleep(5)  # Give some time for the module to power on

            if not setup_lte():
                log("Failed to setup LTE. Saving data and going back to sleep...", level="ERROR")
                save_failed_packet(sensor_data, wakeup_timestamp)
                control_lte_power(state=False)
                continue
                
            #collect data
            sensor_data = collect_sensor_data()
            enable_k96_heatpad_power(False)

            # Send the data using the LTE modem with retry mechanism
            client = CustomMQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, MQTT_TOPIC, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
            if not send_data_with_retry(client, sensor_data, wakeup_timestamp):
                log("Failed to send data after multiple attempts.", level="ERROR")


            control_lte_power(state=False)

        except Exception as e:
            log(f"An error occurred: {e}", level="ERROR")
            control_lte_power(state=False)
            enable_k96_heatpad_power(False)
            # Wait for the next sample interval before retrying
            next_sample_time = math.ceil(utime.time() / config.SAMPLE_INTERVAL) * config.SAMPLE_INTERVAL
            sleep_duration = next_sample_time - utime.time()
            log(f"Sleeping for {sleep_duration} seconds until next sample time due to error...", level="ERROR")
            utime.sleep(sleep_duration)
            continue
        
except KeyboardInterrupt:
    log("KeyboardInterrupt detected. Shutting down gracefully...", level="INFO")
    enable_k96_heatpad_power(False)  # Turn off the K96 sensor and heatpad
    control_lte_power(state=False)  # Power off the LTE modem
    log("Script terminated by user.", level="INFO")

