from servers.utils import SocketManager, PipeManager


def grasp(socket_manager: SocketManager, pipe: PipeManager):
    """
    Grasp an object
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        object_name = response_data['object_name']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute action
    # TODO: remove hard code
    if "yogurt" in object_name.lower():
        object_name = "box_of_yogurt"
    elif "trash" in object_name.lower() and "can" in object_name.lower():
        object_name = "trash_can"
    
    # First, move the gripper to the object
    pipe.write(
        data=("move_gripper_to", object_name),
        role="client"
    )
    while pipe.read(role="client") is None:
        pass

    # Then, close the gripper
    pipe.write(
        data=("close_gripper", ),
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
        'result': "Robot grasped the object successfully."
    })
