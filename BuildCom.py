import socket
import time
import logging
from queue import Queue

def connect_to_RX(client_socket):
    server_ip = "192.168.100.1"
    server_port = 22
    try:
        client_socket.connect((server_ip, server_port))
        client_socket.setblocking(False)
        logging.info("Connection successful and socket set to non-blocking mode")
        return True, client_socket
    except (socket.error, ConnectionError) as e:
        logging.info(f"Connection failed: {e}")
        client_socket.close()
        return False, None

def close_server(client_socket):
    client_socket.close()
    logging.info("Connection closed successfully")

def extract_coordinates(data_str):
    try:
        logging.debug(f"Received data string: {data_str}")
        data_parts = data_str.strip().split(',')
        if len(data_parts) != 4:
            raise ValueError(f"Invalid data format, expected 4 parts but got {len(data_parts)}. Data: {data_parts}")

        timestamp_str = data_parts[0]
        events_str = data_parts[1]
        direction_x_str = data_parts[2]
        direction_y_str = data_parts[3]

        if ':' not in timestamp_str or ':' not in events_str or ':' not in direction_x_str or ':' not in direction_y_str:
            raise ValueError("Invalid data format, expected ':' in each part")

        timestamp = int(timestamp_str.split(':')[1])
        events = int(events_str.split(':')[1])
        direction_x = float(direction_x_str.split(':')[1])
        direction_y = float(direction_y_str.split(':')[1])

        return timestamp, events, direction_x, direction_y
    except (ValueError, IndexError) as e:
        logging.error(f"Error extracting coordinates: {e}")
        return None

def receive_coordinates(client_socket, data_queue: Queue):
    buffer = ""
    client_socket.setblocking(False)
    while True:
        try:
            data = client_socket.recv(4096).decode()  
            if not data:
                time.sleep(0.001)
                continue
            buffer += data

            # Ensure buffer contains complete lines
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line.strip():  # Ensure line is not empty
                    result = extract_coordinates(line)
                    if result:
                        data_queue.put(result)
                        logging.info(f"Received: timestamp={result[0]}, events={result[1]}, direction_x={result[2]}, direction_y={result[3]}")
        except socket.error as e:
            if e.errno != socket.errno.EWOULDBLOCK:
                logging.error(f"Error receiving data: {e}")
                break
    return None
