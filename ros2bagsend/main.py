#!/usr/bin/python

from __future__ import annotations
import rclpy
from rclpy.node import (
    Node,
    MsgType,
    Publisher,
)
from rclpy.parameter import Parameter
import rosbag2_py
import importlib
import copy
from enum import Enum
from typing import List, Dict

class Color(Enum):
    RED            = '\033[31m'
    GREEN          = '\033[32m'
    YELLOW         = '\033[33m'
    BLUE           = '\033[34m'
    RESET          = '\033[0m'

    def __str__(self):
        return self.value

    def __call__(self, s):
        return str(self) + str(s) + str(Color.RESET)

def get_message_type(*, topic_type: str) -> MsgType:
    """
    Get ROS2 message type from string
    """
    package_name, message_type_str = topic_type.split('/', 1)
    module_name, class_name = message_type_str.split('/', 1)
    # Dynamically import modules
    msg_module = importlib.import_module(f"{package_name}.msg")
    # Get Class
    msg_class: MsgType = getattr(msg_module, class_name)
    return msg_class

def main():
    rclpy.init()

    # Create anonymous Node
    node = Node('_bag_publisher_node')

    # Get ROS2 parameters
    node.declare_parameter("bag_file_path", Parameter.Type.STRING)  # bag file path
    node.declare_parameter("start_frame_no", 1)  # start frame number (default: 1)
    node.declare_parameter("end_frame_no", 2147483647)  # end frame number (default: 2147483647)
    node.declare_parameter("statistical_analysis_only", False) # Statistical analysis only

    # bag file path
    bag_file_path: str = node.get_parameter("bag_file_path").get_parameter_value().string_value
    # start frame number
    start_frame_no: int = node.get_parameter("start_frame_no").get_parameter_value().integer_value
    # end frame number
    end_frame_no: int = node.get_parameter("end_frame_no").get_parameter_value().integer_value
    # Statistical analysis only
    statistical_analysis_only: bool = node.get_parameter("statistical_analysis_only").get_parameter_value().bool_value

    # Open bag file
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    rosbag_topic_infos: List[rosbag2_py.TopicMetadata] = reader.get_all_topics_and_types()

    # Create Publisher
    message_types: List[MsgType] = [get_message_type(topic_type=rosbag_topic_info.type) for rosbag_topic_info in rosbag_topic_infos]
    message_names: List[str] = [rosbag_topic_info.name for rosbag_topic_info in rosbag_topic_infos]
    # publishers: {'topic_name': (Publisher object), 'topic_name': (Publisher object), ...}
    publishers: Dict[str, Publisher] = {
        message_name: node.create_publisher(message_type, message_name, 10) for message_type, message_name in zip(message_types, message_names)
    }

    # Statistics Calculation
    if statistical_analysis_only:
        print(f'{Color.BLUE("Statistical information is being analyzed ...")}')
        approximate_frame_count: int = 0
        unused_topic_name: str = None
        while reader.has_next():
            approximate_frame_count += 1 # Topic transmission cycle number (not topic transmission count)
            used_flg: Dict[str, bool] = {message_name: False for message_name in message_names} # Flag initialization

            if approximate_frame_count > end_frame_no:
                break

            if unused_topic_name is not None:
                used_flg[unused_topic_name] = True
                unused_topic_name = None
            while True:
                if not reader.has_next():
                    break
                if sum(used_flg.values()) >= len(message_names):
                    break
                topic_name, data, timestamp = reader.read_next()
                if used_flg[topic_name] == False:
                    used_flg[topic_name] = True
                elif used_flg[topic_name] == True:
                    unused_topic_name = topic_name
                    break
        print(f'{Color.GREEN("topic summary ########################################")}')
        for idx, rosbag_topic_info in enumerate(rosbag_topic_infos):
            print(f'{Color.GREEN("topic_info.name" + str(idx))}: {rosbag_topic_info.name}')
        print(f'{Color.GREEN("======================================================")}')
        print(f'{Color.GREEN("Approximate total frame count")}: {approximate_frame_count - start_frame_no:,}')
        print(f'{Color.GREEN("######################################################")}')
        print(f'{Color.BLUE("Done.")}')
        exit(0)

    unsent_topic_name: str = None
    unsent_data = None
    unsent_timestamp = None
    period_number = 0

    # Sequential reading of bag files
    while reader.has_next():
        period_number += 1 # Topic transmission cycle number (not topic transmission count)

        # Wait for key input
        if period_number >= start_frame_no:
            key = input("Press Enter to publish the next frame... (Interrupted by 'q' key) ")
            if key == 'q':
                break

        published_flg: Dict[str, bool] = {message_name: False for message_name in message_names} # Flag initialization

        if period_number > end_frame_no:
            break

        # Publish if there is data that has not been sent in the previous cycle
        if unsent_topic_name is not None and unsent_data is not None:
            if period_number >= start_frame_no:
                publishers[unsent_topic_name].publish(unsent_data)
                print(f'{Color.YELLOW(period_number)}.{Color.GREEN(unsent_topic_name)}: {unsent_topic_name} {Color.GREEN("time_stamp")}: {unsent_timestamp} ')
            published_flg[unsent_topic_name] = True
            unsent_topic_name = None
            unsent_data = None
            unsent_timestamp = None

        # Send repeated topics in the unit you want to send all at once
        while True:
            if not reader.has_next():
                break

            # After all messages for one cycle are published, go to the next cycle.
            if sum(published_flg.values()) >= len(message_names):
                # Go to the next cycle without publish
                break

            # Read the message
            # e.g.
            #   topic_name: '/zed2i/zed_node/rgb/camera_info'
            #   data: {rawdata}
            #   timestamp: 1705378559012311270
            topic_name, data, timestamp = reader.read_next()

            if published_flg[topic_name] == False:
                # Publish
                if period_number >= start_frame_no:
                    publishers[topic_name].publish(data)
                    print(f'{Color.YELLOW(period_number)}.{Color.GREEN(topic_name)}: {topic_name} {Color.GREEN("time_stamp")}: {timestamp} ')
                # Update flag to Sent
                published_flg[topic_name] = True

            elif published_flg[topic_name] == True:
                # Stored as untransmitted data, transmitted in the next cycle
                unsent_topic_name = topic_name
                unsent_data = copy.deepcopy(data)
                unsent_timestamp = timestamp
                # Go to the next cycle without publish
                break

    # Send any unsent topics
    period_number += 1
    if unsent_topic_name is not None and unsent_data is not None:
        if period_number >= start_frame_no:
            publishers[unsent_topic_name].publish(unsent_data)
            print(f'{Color.YELLOW(period_number)}.{Color.GREEN(unsent_topic_name)}: {unsent_topic_name} {Color.GREEN("time_stamp")}: {unsent_timestamp} ')

    rclpy.shutdown()

if __name__ == '__main__':
    main()