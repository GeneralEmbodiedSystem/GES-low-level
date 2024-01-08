from servers.utils import SocketManager, PipeManager


def get_cam_state(socket_manager: SocketManager, pipe: PipeManager):
    # Send the message to main process
    pipe.write(
        data=("get_cam_state", ),
        role="client"
    )
    
    while True:
        data = pipe.read(role="client")
        if data is not None:
            break

    # Send the response
    socket_manager.write_request({
        'result': data
    })
