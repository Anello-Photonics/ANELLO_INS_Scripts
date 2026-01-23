from pathlib import Path

from rosbags.rosbag1 import Reader as Reader1
from rosbags.typesys import Stores, get_typestore

# import file picker
from tkinter import Tk
from tkinter.filedialog import askopenfilename

from mcap.reader import make_reader
from mcap_ros1.reader import read_ros1_messages
from mcap_ros2.reader import read_ros2_messages

def main():
    # Create a Tkinter root widget
    root = Tk()
    # Hide the main window
    root.withdraw()

    # Ask the user to select a .mcap file
    file_path = askopenfilename(title="Select a .mcap file", filetypes=[("MCAP files", "*.mcap")])
    # file_path = askopenfilename(title="Select a .bag file", filetypes=[("Bag files", "*.bag")])
    # Convert the file path to a Path object
    file_path = Path(file_path)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    topics = [
        "/APINS",
        "/APIMU",
        "/APGPS",
        "/APGP2",
        "/APHDG",
    ]
    output_path_dict = {}
    for topic in topics:
        topic_out_path = file_path.with_name(file_path.stem + f"_{topic[1:]}.csv")
        output_path_dict[topic] = {
            "file_handle" : open(topic_out_path, "w"),
            "header" : [],
            "init" : False
        }

    for msg in read_ros2_messages(file_path):

        topic = msg.channel.topic
        if topic not in topics:
            continue

        fhandle = output_path_dict[topic]["file_handle"]

        if not output_path_dict[topic]["init"]:
            msg_def_lines = msg.schema.data.decode().split("\r\n")
            msg_def_lines = [line for line in msg_def_lines if line.strip() != ""]
            output_path_dict[topic]["header"] = [line.split(" ")[-1] for line in msg_def_lines]

            fhandle.write(",".join(output_path_dict[topic]["header"]))
            fhandle.write("\n")

            output_path_dict[topic]["init"] = True
        
        msg_data = msg.ros_msg

        row = []
        for field in output_path_dict[topic]["header"]:
            row.append(
                str(getattr(msg_data, field))
            )
        
        fhandle.write(",".join(row))
        fhandle.write("\n")


    for topic in topics:
        output_path_dict[topic]["file_handle"].close()

    pass


if __name__=="__main__":
    main()