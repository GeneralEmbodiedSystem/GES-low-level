from servers.utils import SocketManager, PipeManager


def place(socket_manager: SocketManager, pipe: PipeManager):
    """
    Place the object on the hand somewhere
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        place_name = response_data['place_name']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute action
    # TODO: remove hard code
    if "yogurt" in place_name.tolower():
        place_name = "box_of_yogurt"
    elif "trash" in place_name.tolower() and "can" in place_name.tolower():
        place_name = "trash_can"
    
    # First, move the gripper to the place
    pipe.write(
        data=("move_gripper_to", place_name),
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
        'result': "Robot placed the object on the hand successfully."
    })
