#!/home/zwiener/workspace/csapex/devel-debug/lib/csapex/csapex_main
settings:
  - name: additional_args
    type: value
  - name: config
    type: value
    string: /home/zwiener/.csapex/default.apex
  - bool: false
    type: value
    name: config_recovery
  - bool: false
    name: debug
    type: value
  - bool: true
    name: display-frequencies
    type: value
  - bool: false
    name: display-graph-components
    type: value
  - name: display-minimap
    bool: false
    type: value
  - bool: false
    name: display-threads
    type: value
  - string: AdnQywACAAAAAATxAAABlQAADC8AAAYsAAAE8QAAAZUAAAlfAAAFUAAAAAACAAAAB4A=
    name: geometry
    type: value
  - name: grid
    type: value
    bool: false
  - bool: false
    type: value
    name: grid-lock
  - bool: false
    type: value
    name: headless
  - name: ignored_libraries
    type: string_list
    list:
      []
  - type: value
    bool: false
    name: initially_paused
  - is_output: false
    value: /home/zwiener/workspace/csapex/devel-debug/lib/csapex/csapex_main
    is_file: true
    name: path_to_bin
    def: ""
    version: 2
    type: path
    is_input: true
    filter: ""
  - int: 42123
    type: value
    name: port
  - bool: false
    name: schematics
    type: value
  - bool: false
    name: start-server
    type: value
  - bool: true
    type: value
    name: thread_grouping
  - bool: false
    name: threadless
    type: value
  - name: uistate
    type: value
    string: AAAA/wAAAAD9AAAAAwAAAAAAAAAAAAAAAPwCAAAAAvsAAAAKAE4AbwBkAGUAcwAAAAAA/////wAAAMcA////+wAAABQASABlAGwAcABDAGUAbgB0AGUAcgAAAAAA/////wAAAOwA////AAAAAQAAAAAAAAAA/AIAAAAD+wAAAAoARABlAGIAdQBnAAAAAAD/////AAAAdgD////7AAAACABVAG4AZABvAAAAAAD/////AAAAxwD////7AAAAIABUAGgAcgBlAGEAZABNAGEAbgBhAGcAZQBtAGUAbgB0AAAAAAD/////AAAA/QD///8AAAADAAAHPwAAAJX8AQAAAAL7AAAAEABUAGkAbQBlAGwAaQBuAGUAAAAAAAAABz8AAABtAP////sAAAASAFAAcgBvAGYAaQBsAGkAbgBnAAAAAAD/////AAAAjgD///8AAAc/AAAERgAAAAQAAAAEAAAACAAAAAj8AAAAAQAAAAIAAAACAAAACgBUAG8AbwBsAHMBAAAAAP////8AAAAAAAAAAAAAAA4ARgBpAGwAdABlAHIAcwEAAAN6/////wAAAAAAAAAA
nodes:
  - flipped: false
    label: csapex::ImportRos_0
    type: csapex::ImportRos
    pos:
      - -292
      - -32
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    exec_type: 1
    max_frequency: 0
    z: 85
    minimized: false
    uuid: csapex::ImportRos_0
    color:
      - -1
      - -1
      - -1
    state:
      params:
        buffer_size:
          int: 10
          min: 1
          max: 1024
          type: range
          name: buffer_size
          step: 1
        queue:
          type: range
          min: 0
          step: 1
          max: 30
          name: queue
          int: 20
        latch:
          name: latch
          bool: false
          type: value
        refresh:
          type: trigger
          name: refresh
        topic:
          type: set
          name: topic
          txt: /jaco_21_driver/out/joint_state_acc
          string: /jaco_21_driver/out/joint_state_acc
          values:
            -
              - -----
              - -----
            -
              - /clicked_point
              - /clicked_point
            -
              - /estimated_ext_torque
              - /estimated_ext_torque
            -
              - /export
              - /export
            -
              - /initialpose
              - /initialpose
            -
              - /jaco_21_driver/arm_joint_angles/feedback
              - /jaco_21_driver/arm_joint_angles/feedback
            -
              - /jaco_21_driver/arm_joint_angles/result
              - /jaco_21_driver/arm_joint_angles/result
            -
              - /jaco_21_driver/arm_joint_angles/status
              - /jaco_21_driver/arm_joint_angles/status
            -
              - /jaco_21_driver/arm_joint_angles_blocking/feedback
              - /jaco_21_driver/arm_joint_angles_blocking/feedback
            -
              - /jaco_21_driver/arm_joint_angles_blocking/result
              - /jaco_21_driver/arm_joint_angles_blocking/result
            -
              - /jaco_21_driver/arm_joint_angles_blocking/status
              - /jaco_21_driver/arm_joint_angles_blocking/status
            -
              - /jaco_21_driver/finger_joint_angles/feedback
              - /jaco_21_driver/finger_joint_angles/feedback
            -
              - /jaco_21_driver/finger_joint_angles/result
              - /jaco_21_driver/finger_joint_angles/result
            -
              - /jaco_21_driver/finger_joint_angles/status
              - /jaco_21_driver/finger_joint_angles/status
            -
              - /jaco_21_driver/follow_joint_trajectory/manipulator/feedback
              - /jaco_21_driver/follow_joint_trajectory/manipulator/feedback
            -
              - /jaco_21_driver/follow_joint_trajectory/manipulator/result
              - /jaco_21_driver/follow_joint_trajectory/manipulator/result
            -
              - /jaco_21_driver/follow_joint_trajectory/manipulator/status
              - /jaco_21_driver/follow_joint_trajectory/manipulator/status
            -
              - /jaco_21_driver/gripper_command/feedback
              - /jaco_21_driver/gripper_command/feedback
            -
              - /jaco_21_driver/gripper_command/result
              - /jaco_21_driver/gripper_command/result
            -
              - /jaco_21_driver/gripper_command/status
              - /jaco_21_driver/gripper_command/status
            -
              - /jaco_21_driver/out/accelerometers
              - /jaco_21_driver/out/accelerometers
            -
              - /jaco_21_driver/out/finger_positions
              - /jaco_21_driver/out/finger_positions
            -
              - /jaco_21_driver/out/joint_angles
              - /jaco_21_driver/out/joint_angles
            -
              - /jaco_21_driver/out/joint_state_acc
              - /jaco_21_driver/out/joint_state_acc
            -
              - /jaco_21_driver/out/joint_states
              - /jaco_21_driver/out/joint_states
            -
              - /jaco_21_driver/out/sensor_info
              - /jaco_21_driver/out/sensor_info
            -
              - /jaco_21_driver/out/torques_g_free
              - /jaco_21_driver/out/torques_g_free
            -
              - /jaco_21_driver/parameter_descriptions
              - /jaco_21_driver/parameter_descriptions
            -
              - /jaco_21_driver/parameter_updates
              - /jaco_21_driver/parameter_updates
            -
              - /joint_state
              - /joint_state
            -
              - /move_base_simple/goal
              - /move_base_simple/goal
            -
              - /muse_armc_nn/contacts_visualization2
              - /muse_armc_nn/contacts_visualization2
            -
              - /muse_armcl/contacts
              - /muse_armcl/contacts
            -
              - /muse_armcl/contacts_visualization
              - /muse_armcl/contacts_visualization
            -
              - /muse_armcl/particles
              - /muse_armcl/particles
            -
              - /muse_armcl/surface
              - /muse_armcl/surface
            -
              - /muse_armcl_data_publisher/true_contact
              - /muse_armcl_data_publisher/true_contact
            -
              - /muse_armcl_data_publisher/true_contact_visualization
              - /muse_armcl_data_publisher/true_contact_visualization
            -
              - /muse_armcl_nn/cluster_cloud
              - /muse_armcl_nn/cluster_cloud
            -
              - /muse_armcl_nn/contacts
              - /muse_armcl_nn/contacts
            -
              - /muse_armcl_nn/contacts_visualization
              - /muse_armcl_nn/contacts_visualization
            -
              - /muse_armcl_nn/particles
              - /muse_armcl_nn/particles
            -
              - /muse_armcl_nn/surface
              - /muse_armcl_nn/surface
            -
              - /rosout
              - /rosout
            -
              - /rosout_agg
              - /rosout_agg
            -
              - /tf
              - /tf
            -
              - /tf_static
              - /tf_static
            -
              - /true_contacts
              - /true_contacts
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
          step: 0.1
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          max: 10
          double: 1
          min: 0
          step: 0.1
        import_mode:
          int: 1
          values:
            -
              - BackAndLatch
              - 0
            -
              - FrontAndPop
              - 1
          txt: FrontAndPop
          type: set
          name: import_mode
      persistent_params:
        []
  - max_frequency: 0
    uuid: csapex::Jaco2ResidualVectorNode_0
    type: csapex::Jaco2ResidualVectorNode
    label: csapex::Jaco2ResidualVectorNode_0
    pos:
      - 157
      - -167
    color:
      - -1
      - -1
      - -1
    z: 73
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        invert_gravity:
          type: value
          name: invert_gravity
          bool: false
        reset:
          type: trigger
          name: reset
        use_dynamic_calibration:
          type: value
          name: use_dynamic_calibration
          bool: true
        static_base:
          type: value
          name: static_base
          bool: false
        vector_in_reset:
          name: vector_in_reset
          type: value
          bool: false
        ~model/gain_0:
          type: value
          name: ~model/gain_0
          double: 10
        ~model/number_of_gains:
          type: range
          name: ~model/number_of_gains
          int: 1
          min: 1
          max: 6
          step: 1
        ~model/robot_model:
          type: value
          name: ~model/robot_model
          string: robot_description
        ~model/acc_transforms:
          value: ""
          type: path
          name: ~model/acc_transforms
          version: 2
          def: ""
          filter: ""
          is_file: true
          is_input: true
          is_output: false
        ~model/chain_root:
          type: value
          name: ~model/chain_root
          string: jaco_link_base
        ~model/chain_tip:
          type: value
          name: ~model/chain_tip
          string: jaco_link_hand
        ~model/dynamic_calibration:
          version: 2
          value: /home/zwiener/contact_evo/configs/regression_rb_param_better.txt
          is_input: true
          def: ""
          filter: ""
          is_file: true
          is_output: false
          type: path
          name: ~model/dynamic_calibration
      persistent_params:
        []
  - z: 61
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 0
    logger_level: 1
    state:
      params:
        frame_id:
          type: value
          name: frame_id
          string: csapex
        set_frame_id:
          type: value
          name: set_frame_id
          bool: false
        use_current_time:
          type: value
          name: use_current_time
          bool: false
      persistent_params:
        []
    type: csapex::ConvertJaco2JointState2SensorMsgsJointState
    uuid: csapex::ConvertJaco2JointState2SensorMsgsJointState_0
    max_frequency: 0
    label: csapex::ConvertJaco2JointState2SensorMsgsJointState_0
    pos:
      - 587
      - 7
    color:
      - -1
      - -1
      - -1
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_0
    max_frequency: 0
    label: csapex::ExportRos_0
    pos:
      - 1289
      - -60
    color:
      - -1
      - -1
      - -1
    z: 58
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    exec_type: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /estimated_ext_torque
      persistent_params:
        []
threads:
  assignments:
    - uuid: ~
      id: 1
    - uuid: csapex::ImportRos_0
      id: 1
    - uuid: csapex::Jaco2ResidualVectorNode_0
      id: 1
    - uuid: csapex::ConvertJaco2JointState2SensorMsgsJointState_0
      id: 1
    - uuid: csapex::ExportRos_0
      id: 1
  private_affinity:
    - true
    - true
    - true
    - true
  groups:
    - affinity:
        - true
        - true
        - true
        - true
      name: default
      id: 1
adapters:
  []
connections:
  - uuid: csapex::Jaco2ResidualVectorNode_0:|:out_0
    targets:
      - csapex::ConvertJaco2JointState2SensorMsgsJointState_0:|:in_0
    types:
      - default
  - uuid: csapex::ConvertJaco2JointState2SensorMsgsJointState_0:|:out_0
    targets:
      - csapex::ExportRos_0:|:in_0
    types:
      - default
  - uuid: csapex::ImportRos_0:|:out_0
    targets:
      - csapex::Jaco2ResidualVectorNode_0:|:in_0
    types:
      - default
version:
  minor: 9
  patch: 7
  major: 0
uuid_map:
  csapex::ConvertJaco2JointState2SensorMsgsJointState: 1
  csapex::FileImporter: 1
  csapex::ExportRos: 2
  csapex::CombineExtJointStateMessage: 1
  csapex::ExportCout: 1
  csapex::Latch: 3
  csapex::SplitExtJointStateMessage: 1
  csapex::ImportRos: 2
  csapex::Jaco2ResidualVectorNode: 1