from mlx.mlx90640 import Mlx9064x
import sys
import time


def main():
    if len(sys.argv) <= 1:
        print("Usage:")
        print("python3 mlx90640_dump_frame.py <serial_port> <read_frames_count> <frame_rate>\n")
        print(
            "The first argument needs to be the name of the COM port to connect (e.g. /dev/ttyUSB0, /dev/ttyACM0, COM3)")
        print("  This argument can also be 'auto', works only when 1 comport on system\n")
        print("examples:")
        print(" - python3 mlx90640_dump_frame.py COM20             ==> 1 frame at 8 fps")
        print(" - python3 mlx90640_dump_frame.py auto              ==> 1 frame at 8 fps")
        print(" - python3 mlx90640_dump_frame.py COM20 10          ==> 10 frames at 8 fps")
        print(" - python3 mlx90640_dump_frame.py COM20 10 16       ==> 10 frames at 16 fps")
        print(" - python3 mlx90640_dump_frame.py 1000              ==> 1000 frames at 8 fps")
        print(" - python3 mlx90640_dump_frame.py 100 1             ==> 100 frames at 1 fps")
        print(" - python3 mlx90640_dump_frame.py 0                 ==> 0 frames -> find out which comport is assigned")
        print("")
        exit(1)

    # defaults
    frame_rate = 8.0
    max_frames = 1
    port = 'auto'
    try:
        max_frames = int(sys.argv[1])
        if len(sys.argv) >= 3:
            frame_rate = float(sys.argv[2])
    except ValueError:
        port = sys.argv[1]
        if len(sys.argv) >= 3:
            max_frames = int(sys.argv[2])
        if len(sys.argv) >= 4:
            frame_rate = float(sys.argv[3])
        pass

    dev = Mlx9064x(port, frame_rate=frame_rate)
    dev.init()
    dev.emissivity = 0.95

    frame_count = 0
    while frame_count < max_frames:
        frame = None
        try:
            frame = dev.read_frame()
        except Exception as e:
            print("ERROR:", e)
            dev.clear_error(frame_rate)
            pass

        if frame is not None:
            f = dev.do_compensation(frame, add_ambient_temperature=True)
            f = dev.do_handle_bad_pixels(f)
            amb = f.pop() # pop ambient temperature from the array.
            print(">", frame_count, ": AMB: {:6.2f} Array: ".format(amb) + ",".join(map("{:.2f}".format, f)))
            frame_count += 1
        time.sleep(0.001)


if __name__ == '__main__':
    main()
