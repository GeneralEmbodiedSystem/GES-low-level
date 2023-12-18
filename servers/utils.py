import socket
import struct
from typing import Any, Iterable, Optional, List
import os
import mmap
import random
import string
import json
from io import BytesIO

try:
    from typing import Literal
except ImportError:
    from typing_extensions import Literal

import numpy as np


class PipeManager:
    """
    Class for managing pipe communication (send and receive).
    The communication is based on shared memory.
    The numpy array is not supported.
    """
    def __init__(
        self,
        tmp_path: str,
        name: str,
        size: int,
        create: bool = False
    ):
        self.tmp_path = tmp_path
        self.name = name
        self.size = size

        # Create a file to back the shared memory
        path = os.path.join(tmp_path, name)
        path = os.path.expanduser(path)
        self.shm_file_path = path

        if create:
            # Create the file
            if not os.path.exists(path):
                # mkdir
                os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                # Allocate space
                f.seek(size - 1)
                f.write(b'\x00')
        
        with open(self.shm_file_path, "r+b") as f:
            self.shm = mmap.mmap(f.fileno(), size)
    
    def close(self, remove=False):
        self.shm.close()
        if remove:
            os.remove(self.shm_file_path)
    
    def read_signal(self, role: Literal["server", "client"]) -> bool:
        """
        Read the signal from the shared memory (first 2 bytes).
        When `role='server'`, the signal should represent whether
        the server has ready sent the data.
        When `role='client'`, the signal should represent whether
        the client has ready sent the data.
        """
        if role == "server":
            self.shm.seek(0)
        elif role == "client":
            self.shm.seek(1)
        else:
            raise ValueError("check should be 'server' or 'client'")
        signal = self.shm.read(1)
        signal = bool.from_bytes(signal, "big")
        
        return signal

    def write_signal(self, signal: bool, role: Literal["server", "client"]) -> None:
        """
        Write the signal to the shared memory (first 2 bytes).
        When `role='server'`, the signal should represent whether
        the server has ready sent the data.
        When `role='client'`, the signal should represent whether
        the client has ready sent the data.
        """
        if role == "server":
            self.shm.seek(0)
        elif role == "client":
            self.shm.seek(1)
        else:
            raise ValueError("check should be 'server' or 'client'")
        self.shm.write(signal.to_bytes(1, byteorder="big"))
    
    def read_request(self) -> Any:
        """
        Read the request data from the shared memory.
        """
        # Read the length of the pickled data
        self.shm.seek(2)
        length_bytes = self.shm.read(4)
        length = int.from_bytes(length_bytes, byteorder='big')
        # Now read exactly 'length' bytes
        json_req = self.shm.read(length)
        req = json.loads(json_req.decode())

        return req
    
    def write_request(self, req: Any) -> None:
        """
        Write the request data to the shared memory.
        """
        json_req = json.dumps(req)
        # Write the length of the pickled data first
        self.shm.seek(2)
        self.shm.write(len(json_req).to_bytes(4, byteorder='big'))
        self.shm.write(json_req.encode())


class SocketManager:
    """
    Class for managing socket communication (send and receive).
    The communication is based on shared memory and socket.
    Socket for sending and receiving json data while
    shared memory for sending and receiving numpy array.

    Client Example:
    ```
    sm = SocketManager(...)
    sm.init_shared_memory()
    with sm:
        sm.send_shared_memory_name()
        sm.send_string(string)
        sm.send_float(float)
        ...
    sm.close_shared_memory(remove=True)
    ```
    
    Server Example:
    ```
    sm = SocketManager(...)
    sm.set_socket(client_socket)
    name = sm.recv_shared_memory_name()
    sm.init_shared_memory(name=name, create=False, size=4*1024*1024)
    string = sm.recv_string()
    ...
    sm.close_shared_memory()
    ```
    """
    def __init__(
        self, 
        server_address: Iterable[Any] = ("localhost", 8000),
        tmp_path: Optional[str] = None
    ):
        self.tmp_path = tmp_path
        
        self.host, self.port = server_address
        self.shm = None
        self.socket = None

    def __enter__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.socket.close()
    
    def set_socket(self, socket: socket.socket):
        self.socket = socket
    
    def init_shared_memory(self, name=None, create=True, size=1024*1024*1024):
        if create:
            # Random generated name
            # 6 digits or letters
            chars = string.ascii_letters + string.digits
            name = name or ''.join(random.choice(chars) for _ in range(6))

            # Create a file to back the shared memory
            path = os.path.join(self.tmp_path, name)
            path = os.path.expanduser(path)
            with open(path, "wb") as f:
                # Allocate space
                f.seek(size - 1)
                f.write(b'\x00')
        
        self.shm_name = name
        self.shm_size = size
        self.shm_file_path = os.path.join(self.tmp_path, name)
        self.shm_file_path = os.path.expanduser(self.shm_file_path)
        with open(self.shm_file_path, "r+b") as f:
            self.shm = mmap.mmap(f.fileno(), size)
    
    def close_shared_memory(self, remove=False):
        if self.shm is None:
            raise ValueError("shared_memory is not initialized")
        self.shm.close()
        if remove:
            os.remove(self.shm_file_path)
        
    def set_shared_memory_cursor(self, offset: int):
        if self.shm is None:
            raise ValueError("shared_memory is not initialized")
        self.shm.seek(offset)
    
    def send_shared_memory_name(self):
        if self.shm is None:
            raise ValueError("shared_memory is not initialized")
        self.send_string(self.shm_name)
    
    def recv_shared_memory_name(self):
        name = self.recv_string()
        return name

    def send_string(self, string: str, len_bytes=8) -> None:
        """
        Send a string by the socket (length takes 8 bytes by default).
        """
        # Send the length of the string
        string_length = len(string).to_bytes(len_bytes, byteorder="big")
        self.socket.sendall(string_length)

        # Send the string
        self.socket.sendall(string.encode())

    def recv_string(self, len_bytes=8) -> str:
        """
        Receive a string by the socket (length takes 8 bytes by default).
        """
        # Receive the length of the string
        string_length = self.socket.recv(len_bytes)
        string_length = int.from_bytes(string_length, "big")

        # Receive the string
        string = b''
        while len(string) < string_length:
            recv_size = min(4096, string_length - len(string))
            packet = self.socket.recv(recv_size)
            string += packet

        # Decode the string
        string = string.decode()

        return string

    def send_float(self, value: float) -> None:
        """
        Send a float by the socket.
        """
        # Send the float
        self.socket.sendall(struct.pack('>d', value))

    def recv_float(self) -> float:
        """
        Receive a float by the socket.
        """
        # Receive the float
        value = struct.unpack('>d', self.socket.recv(8))[0]

        return value

    def send_int(self, value: int, len_bytes=4) -> None:
        """
        Send an integer by the socket (use 4 bytes by default).
        """
        # Send the integer
        self.socket.sendall(value.to_bytes(len_bytes, byteorder="big"))

    def recv_int(self, len_bytes=4) -> int:
        """
        Receive an integer by the socket (use 4 bytes by default).
        """
        # Receive the integer
        value = self.socket.recv(len_bytes)
        value = int.from_bytes(value, "big")

        return value

    def send_func_code(self, func_code: int) -> None:
        """
        Send a function code by the socket.
        """
        # Send the function code
        self.send_int(func_code)
    
    def recv_func_code(self) -> int:
        """
        Receive a function code by the socket.
        """
        # Receive the function code
        func_code = self.recv_int()

        return func_code

    def send_bool(self, value: bool) -> None:
        """
        Send a boolean by the socket.
        """
        # Send the boolean
        self.socket.sendall(value.to_bytes(1, byteorder="big"))

    def recv_bool(self) -> bool:
        """
        Receive a boolean by the socket.
        """
        # Receive the boolean
        value = self.socket.recv(1)
        value = bool.from_bytes(value, "big")

        return value
    
    @staticmethod
    def serialize_array(array: np.ndarray, np_bytes: BytesIO) -> None:
        """
        Serialize a numpy array.
        """
        dtype_bytes = str(array.dtype).encode()
        len_dtype_bytes = len(dtype_bytes).to_bytes(4, byteorder="big")
        np_bytes.write(len_dtype_bytes)
        np_bytes.write(dtype_bytes)

        shape_bytes = np.array(array.shape, dtype=np.int32).tobytes()
        len_shape_bytes = len(shape_bytes).to_bytes(4, byteorder="big")
        np_bytes.write(len_shape_bytes)
        np_bytes.write(shape_bytes)

        array_bytes = array.tobytes()
        np_bytes.write(array_bytes)
    
    @staticmethod
    def deserialize_array(np_bytes: BytesIO) -> np.ndarray:
        """
        Deserialize a numpy array.
        """
        # Read the length of the dtype
        len_dtype_bytes = np_bytes.read(4)
        len_dtype = int.from_bytes(len_dtype_bytes, byteorder="big")
        # Read the dtype
        dtype_bytes = np_bytes.read(len_dtype)
        dtype = dtype_bytes.decode()

        # Read the length of the shape
        len_shape_bytes = np_bytes.read(4)
        len_shape = int.from_bytes(len_shape_bytes, byteorder="big")
        # Read the shape
        shape_bytes = np_bytes.read(len_shape)
        shape = np.frombuffer(shape_bytes, dtype=np.int32)

        # Calculate the length of the array bytes
        array_length = np.prod(shape) * np.dtype(dtype).itemsize
        # Read the array bytes
        array_bytes = np_bytes.read(array_length)
        array = np.frombuffer(array_bytes, dtype=dtype)
        array = array.reshape(shape)

        return array
    
    def write_request(self, req: dict) -> int:
        """
        Write the request data to the shared memory. The file cursor to
        the shared memory WILL be reset before writing. The return value
        is the length of the written data.

        Note: Do not put np.ndarray in the request data.
        Instead, use `write_arrays()` and `read_arrays()`, and put
        the index of the array in the request data.

        Example:
        ```
        socket_manager.write_request({
            "array_index": 0,
            "other_data": ...
        })
        socket_manager.write_arrays([array])
        ```
        """
        json_req = json.dumps(req)
        tot_len = 0
        # Write the length of the pickled data first
        self.shm.seek(0)
        tot_len += self.shm.write(len(json_req).to_bytes(4, byteorder='big'))
        tot_len += self.shm.write(json_req.encode())
        # Finish signal
        self.send_bool(True)

        return tot_len

    def read_request(self) -> dict:
        """
        Read the request data from the shared memory. The file cursor to
        the shared memory WILL be reset before reading.

        Note: Do not put np.ndarray in the request data.
        Instead, use `write_arrays()` and `read_arrays()`, and put
        the index of the array in the request data.
        
        Example:
        ```
        req = socket_manager.read_request()
        arrays = socket_manager.read_arrays()
        array = arrays[req["array_index"]]
        ```
        """
        # Finish signal
        self.recv_bool()
        self.shm.seek(0)
        # Read the length of the pickled data
        length_bytes = self.shm.read(4)
        length = int.from_bytes(length_bytes, byteorder='big')
        # Now read exactly 'length' bytes
        json_req = self.shm.read(length)
        req = json.loads(json_req.decode())

        return req
    
    def write_arrays(self, arrays: List[np.ndarray], 
                     write=True, send_signal=True) -> None:
        """
        Write the arrays to the shared memory. The file cursor to
        the shared memory will NOT be reset before writing.
        
        Parameter:
            write: If True, write the arrays to the shared memory.
            send_signal: If True, send the finish signal.

            Note: This two parameters are useful when the array writing
            and the request sending are in different places or processes.
        """
        if write:
            # Write the number of arrays
            self.shm.write(len(arrays).to_bytes(4, byteorder='big'))
            # Write the arrays
            for array in arrays:
                self.serialize_array(array, self.shm)
        if send_signal:
            # Finish signal
            self.send_bool(True)
    
    def read_arrays(self) -> List[np.ndarray]:
        """
        Read the arrays from the shared memory. The file cursor to
        the shared memory will NOT be reset before reading.
        """
        # Finish signal
        self.recv_bool()
        # Read the number of arrays
        num_arrays = int.from_bytes(self.shm.read(4), byteorder='big')
        # Read the arrays
        arrays = []
        for _ in range(num_arrays):
            array = self.deserialize_array(self.shm)
            arrays.append(array)
        
        return arrays
