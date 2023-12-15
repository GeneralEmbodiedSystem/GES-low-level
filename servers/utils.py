import socket
import struct
from typing import Any, Callable, Iterable, Optional
from multiprocessing import shared_memory

import numpy as np


class SocketManager:
    """
    Class for managing socket communication (send and receive).

    Client Example:
    ```
    sm = SocketManager(...)
    sm.init_shared_memory()
    with sm:
        sm.send_shared_memory_name()
        sm.send_array(array)
        sm.send_string(string)
        sm.send_float(float)
        ...
    sm.close_shared_memory()
    ```
    
    Server Example:
    ```
    sm = SocketManager(...)
    sm.set_socket(client_socket)
    name = sm.recv_shared_memory_name()
    sm.init_shared_memory(name=name, create=False)
    array = sm.recv_array()
    string = sm.recv_string()
    ...
    sm.close_shared_memory()
    ```
    """
    def __init__(
        self, 
        shared_memory: Optional[bool] = False,
        shared_memory_size: Optional[int] = 4096,
        server_address: Iterable[Any] = ("localhost", 8000)
    ):
        self.shared_memory = shared_memory
        self.shared_memory_size = shared_memory_size
        
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
    
    def init_shared_memory(self, name=None, create=True):
        if not self.shared_memory:
            raise ValueError("shared_memory is not enabled")
        self.shm = shared_memory.SharedMemory(
            name=name, create=create,
            size=self.shared_memory_size)
    
    def close_shared_memory(self):
        if self.shm is None:
            raise ValueError("shared_memory is not initialized")
        self.shm.close()
        self.shm.unlink()

    def send_shared_memory_name(self):
        if self.shm is None:
            raise ValueError("shared_memory is not initialized")
        self.send_string(self.shm.name)
    
    def recv_shared_memory_name(self):
        if not self.shared_memory:
            raise ValueError("shared_memory is not enabled")
        name = self.recv_string()
        return name

    def write_shared_array(self, image: np.ndarray) -> str:
        # Create a NumPy array backed by shared memory
        shared_arr = np.ndarray(image.shape, dtype=image.dtype, buffer=self.shm.buf)
        # Copy the array to shared memory
        np.copyto(shared_arr, image)

    def read_shared_array(self, shape: tuple, dtype: np.dtype) -> np.ndarray:
        # Create a NumPy array backed by shared memory
        shared_arr = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)

        return shared_arr

    def send_array(self, array: np.ndarray) -> None:
        """
        Send a ndarray by the socket.
        """
        # Send the array dtype
        dtype_str = str(array.dtype)
        dtype_bytes = dtype_str.encode()
        self.socket.sendall(len(dtype_bytes).to_bytes(4, byteorder="big"))
        self.socket.sendall(dtype_bytes)
        # Send the array dimension
        self.socket.sendall(
            np.array(array.shape, dtype=np.int32).tobytes())
        # Send the array
        self.write_shared_array(array)
        # Send the finish signal
        self.socket.sendall(b"ok")

    def recv_array(self) -> np.ndarray:
        """
        Receive a ndarray by the socket.
        """
        # Receive the array dtype
        dtype_length = int.from_bytes(self.socket.recv(4), "big")
        dtype = self.socket.recv(dtype_length).decode()
        # Receive the array dimension
        shape = self.socket.recv(12)
        shape = np.frombuffer(shape, dtype=np.int32)
        # Receive the finish signal of the array copy
        self.socket.recv(2)
        # Receive the array
        array = self.read_shared_array(shape, dtype).copy()

        return array

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

    def send_tuple(self, tup: tuple) -> None:
        """
        Send a tuple by the socket.

        Note: the element types can be different for tuples.
        """
        # Check if the tuple is empty
        if len(tup) == 0:
            return

        for element in tup:
            send_fn = self.get_send_fn(type(element))
            send_fn(element)

    def recv_tuple(self, template: tuple) -> tuple:
        """
        Receive a tuple by the socket.

        Note: the element types can be different for tuples.
        """
        # Check if the tuple is empty
        if len(template) == 0:
            return ()

        tup = []
        for element in template:
            recv_fn = self.get_recv_fn(type(element))
            if recv_fn == self.recv_list or recv_fn == self.recv_tuple \
                or recv_fn == self.recv_dict:
                element = recv_fn(element)
            else:
                element = recv_fn()
            tup.append(element)

        return tuple(tup)

    def send_list(self, lst: list, len_bytes=4) -> None:
        """
        Send a list by the socket (length takes 4 bytes by default).

        Note: the type of the elements of the list must be the SAME.
        """
        # Send the length of the list
        lst_length = len(lst).to_bytes(len_bytes, byteorder="big")
        self.socket.sendall(lst_length)

        # Check if the list is empty
        if len(lst) == 0:
            return

        # Get the type of the elements of the list
        type_ = type(lst[0])
        send_fn = self.get_send_fn(type_)

        # Send the elements of the list
        for element in lst:
            send_fn(element)

    def recv_list(self, template: list, len_bytes=4) -> list:
        """
        Receive a list by the socket (length takes 4 bytes by default).

        template: a template list that contains the type of the elements of the list.

        Note: the type of the elements of the list must be the SAME.
        """
        # Receive the length of the list
        lst_length = self.socket.recv(len_bytes)
        lst_length = int.from_bytes(lst_length, "big")

        # Check if the list is empty
        if lst_length == 0:
            return []

        # Receive the elements of the list
        lst = []
        element_type = type(template[0])
        recv_fn = self.get_recv_fn(element_type)
        
        for _ in range(lst_length):
            if recv_fn == self.recv_list or recv_fn == self.recv_tuple \
                or recv_fn == self.recv_dict:
                element = recv_fn(template[0])
            else:
                element = recv_fn()
            lst.append(element)

        return lst

    def send_dict(self, dic: dict, len_bytes=4) -> None:
        """
        Send a json dictionary by the socket (length takes 4 bytes by default).

        Note: the type of the keys must be STRING.
        """
        # Send the length of the dictionary as a 8-byte string
        dic_length = len(dic).to_bytes(len_bytes, byteorder="big")
        self.socket.sendall(dic_length)

        # Check if the dictionary is empty
        if len(dic) == 0:
            return

        # Send the keys and values of the dictionary
        for key, value in dic.items():
            if type(key) is not str:
                raise TypeError(f"Unsupported key type: {type(key)}")
            self.send_string(key, len_bytes=4)
            send_fn = self.get_send_fn(type(value))
            send_fn(value)

    def recv_dict(self, template: dict, len_bytes=4) -> dict:
        """
        Receive a json dictionary by the socket (length takes 4 bytes by default).

        template: a template dictionary that contains the type of the values of the dictionary.

        Note: the type of the keys must be STRING.
        """
        # Receive the length of the dictionary
        dic_length = self.socket.recv(len_bytes)
        dic_length = int.from_bytes(dic_length, "big")

        # Check if the dictionary is empty
        if dic_length == 0:
            return {}

        # Receive the keys and values of the dictionary
        dic = {}
        for _ in range(dic_length):
            key = self.recv_string(len_bytes=4)
            value = template[key]
            recv_fn = self.get_recv_fn(type(value))
            if recv_fn == self.recv_list or recv_fn == self.recv_tuple \
                or recv_fn == self.recv_dict:
                value = recv_fn(value)
            else:
                value = recv_fn()
            dic[key] = value

        return dic

    def get_send_fn(self, type_: Any) -> Callable:
        """
        Get the send function for the given type.
        """
        if type_ == np.ndarray:
            return self.send_array
        elif type_ == str:
            return self.send_string
        elif type_ == float:
            return self.send_float
        elif type_ == int:
            return self.send_int
        elif type_ == bool:
            return self.send_bool
        elif type_ == list:
            return self.send_list
        elif type_ == tuple:
            return self.send_tuple
        elif type_ == dict:
            return self.send_dict
        else:
            raise TypeError(f"Unsupported type: {type_}")

    def get_recv_fn(self, type_: Any) -> Callable:
        """
        Get the receive function for the given type.
        """
        if type_ == np.ndarray:
            return self.recv_array
        elif type_ == str:
            return self.recv_string
        elif type_ == float:
            return self.recv_float
        elif type_ == int:
            return self.recv_int
        elif type_ == bool:
            return self.recv_bool
        elif type_ == list:
            return self.recv_list
        elif type_ == tuple:
            return self.recv_tuple
        elif type_ == dict:
            return self.recv_dict
        else:
            raise TypeError(f"Unsupported type: {type_}")
