from servers.utils import SocketManager, PipeManager


def place(socket_manager: SocketManager, pipe: PipeManager):
    """
    Place the object on the hand somewhere
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        target_position = response_data['target_position']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute action
    # TODO: remove hard code
    if "yogurt" in target_position.lower():
        target_position = "box_of_yogurt"
    elif "trash" in target_position.lower() and "can" in target_position.lower():
        target_position = "trash_can"
    
    # First, move the gripper to the place
    pipe.write(
        data=("move_gripper_to", target_position),
        role="client"
    )
    while pipe.read(role="client") is None:
        pass

    # Then, open the gripper
    pipe.write(
        data=("open_gripper", ),
        role="client"
    )
    while pipe.read(role="client") is None:
        pass

    # Finally, raise the gripper
    pipe.write(
        data=("raise_gripper", ),
        role="client"
    )
    while pipe.read(role="client") is None:
        pass

    # Send the response
    socket_manager.write_request({
        'result': "Robot placed the object successfully."
    })
