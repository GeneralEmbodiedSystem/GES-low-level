class BodySimulatorCfg:
    scene_cfg = dict(
        type="InteractiveTraversableScene",
        scene_model="Beechwood_0_int",
        load_object_categories=["floors", "walls", "ceilings"],  # this line means load_mode == "Quick". Comment it if you want to load all objects
    )
    
    robot0_cfg = dict(
        # type="Fetch",
        type="TiagoAdvanced",
        obs_modalities=["scan", "rgb", "depth"],
        action_type="continuous",
        action_normalize=True,
        grasping_mode="sticky",
        position=[-4.0, 0.0, 0.0],
    )
    
    objects_cfg = [ 
        dict(
            type="DatasetObject",
            name="delicious_apple",
            category="apple",
            model="agveuv",
            size=0.07,
            position=[0.83, -0.1, 0.97],
        ),  # apple
        dict(
            type="DatasetObject",
            name="alarm_clock",
            category="alarm_clock",
            model="cvknrh",
            size=0.06,
            position=[1.53, 0.1, 0.97],
        ),  # alarm_clock
        dict(
            type="DatasetObject",
            name="water_bottle",
            category="water_bottle",
            model="ackxiy",
            size=0.06,
            position=[1.33, -0.5, 0.97],
        ),  # alarm_clock
        {
            "type": "LightObject", 
            "name": "brilliant_light",
            "light_type": "Sphere", 
            "intensity": 50000, 
            "radius": 0.1, 
            "position": [3.0, 3.0, 4.0],
        },
        dict(
            type="DatasetObject",
            name="table",
            category="breakfast_table",
            model="lcsizg",
            bounding_box=[0.5, 0.5, 0.8],
            fit_avg_dim_volume=False,
            fixed_base=True,
            position=[0.7, -0.1, 0.6],
            orientation=[0, 0, 0.707, 0.707],
        ),  # table
        dict(
            type="DatasetObject",
            name="chair",
            category="straight_chair",
            model="amgwaw",
            bounding_box=None,
            fit_avg_dim_volume=True,
            fixed_base=False,
            position=[-0.45, 0.65, 0.425],
            orientation=[0, 0, -0.9990215, -0.0442276],
        ),  # chair
        dict(
            type="PrimitiveObject",
            name="box",
            primitive_type="Cube",
            rgba=[1.0, 0, 0, 1.0],
            size=0.05,
            position=[0.53, -0.1, 0.97],
        ),  # box
    ]
    # 整体上让x 和 y小一些，可以让robot和物体放到客厅里去。现在的房间太小了
    