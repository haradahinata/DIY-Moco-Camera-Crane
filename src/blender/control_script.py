import bpy
import serial
import time
import pygame
import math
import serial.tools.list_ports

VID = 0x1a86
PID = 0x7523

# シリアルポート自動検出
ports = serial.tools.list_ports.comports()
ser = None
for port in ports:
    if port.vid == VID and port.pid == PID:
        ser = serial.Serial(port.device, 115200)
        time.sleep(0.5)
        break

pygame.init()

# ゲームパッドが接続されているか確認
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    joystick_connected = True
else:
    joystick = None
    joystick_connected = False
    
# ジョイスティックのデッドゾーン
deadzone_threshold = 0.2

# focusのZ回転速度
focus_rotation_speed = 0.05

last_rotation_deg_y = None
last_rotation_deg_x = None
last_arm_rotation_x = None
last_arm_rotation_z = None
last_focus_rotation_z = None

# 角度の最小値と最大値を各軸ごとに設定
angle_limits = {
    "camera_x": (-90.0, 90.0),
    "camera_y": (-120.0, 120.0),
    "arm_x": (-42.0, 42.0),
    "arm_z": (-180.0, 180.0),
    "focus_z": (0.0, 360.0),
}

def rescale_deadzone(value, deadzone):
# デッドゾーンを超えた値を0〜1にリスケーリング
    if value > 0:
        return (value - deadzone) / (1 - deadzone)
    elif value < 0:
        return (value + deadzone) / (1 - deadzone)
    else:
        return 0.0

def apply_deadzone(value):
# ジョイスティックの入力値にデッドゾーンを適用し、リスケーリング
    if abs(value) < deadzone_threshold:
        return 0.0
    return rescale_deadzone(value, deadzone_threshold)

def send_rotation(rotation_deg_x, rotation_deg_y, arm_rotation_x, arm_rotation_z, focus_rotation_z):
    message = f"ANGLE:{arm_rotation_z},{arm_rotation_x},{rotation_deg_y},{rotation_deg_x},{focus_rotation_z}\n"
    print(f"Sending angles: {message.strip()}")
    ser.write(message.encode())

def handle_gamepad_input():
    global last_rotation_deg_x, last_rotation_deg_y, last_arm_rotation_x, last_arm_rotation_z, last_focus_rotation_z

    pygame.event.pump()

    # オブジェクトの取得
    obj = bpy.data.objects['Camera']
    armature = bpy.data.objects['アーマチュア']
    bone = armature.pose.bones['arm']
    focus = bpy.data.objects['focus']

    if joystick_connected:
        # カメラの回転操作
        x_axis = apply_deadzone(joystick.get_axis(3))
        y_axis = apply_deadzone(joystick.get_axis(2))

        obj.rotation_euler.x += x_axis * -0.02
        obj.rotation_euler.y += y_axis * -0.02

        # armの回転操作
        arm_x_axis = apply_deadzone(joystick.get_axis(1))
        arm_z_axis = apply_deadzone(joystick.get_axis(0))

        bone.rotation_euler.x += arm_x_axis * -0.01
        bone.rotation_euler.z += arm_z_axis * -0.01

        # focusのZ回転操作
        if joystick.get_button(12):
            focus.rotation_euler.z -= focus_rotation_speed
        if joystick.get_button(11):
            focus.rotation_euler.z += focus_rotation_speed

    # カメラの角度を取得し、度数に変換
    rotation_deg_y = obj.rotation_euler.y * 180.0 / math.pi
    rotation_deg_x = obj.rotation_euler.x * 180.0 / math.pi

    # armの角度を取得し、度数に変換
    arm_rotation_x = bone.rotation_euler.x * 180.0 / math.pi
    arm_rotation_z = bone.rotation_euler.z * 180.0 / math.pi

    # focusの角度を取得し、度数に変換
    focus_rotation_z = focus.rotation_euler.z * 180.0 / math.pi

    # 角度を制限
    rotation_deg_y = max(min(rotation_deg_y, angle_limits["camera_y"][1]), angle_limits["camera_y"][0])
    rotation_deg_x = max(min(rotation_deg_x, angle_limits["camera_x"][1]), angle_limits["camera_x"][0])
    arm_rotation_x = max(min(arm_rotation_x, angle_limits["arm_x"][1]), angle_limits["arm_x"][0])
    arm_rotation_z = max(min(arm_rotation_z, angle_limits["arm_z"][1]), angle_limits["arm_z"][0])
    focus_rotation_z = max(min(focus_rotation_z, angle_limits["focus_z"][1]), angle_limits["focus_z"][0])

    # 制限した角度をラジアンに戻して適用
    obj.rotation_euler.y = rotation_deg_y * math.pi / 180.0
    obj.rotation_euler.x = rotation_deg_x * math.pi / 180.0
    bone.rotation_euler.x = arm_rotation_x * math.pi / 180.0
    bone.rotation_euler.z = arm_rotation_z * math.pi / 180.0
    focus.rotation_euler.z = focus_rotation_z * math.pi / 180.0

    # 必要に応じて角度を送信
    if (last_rotation_deg_x is None or abs(rotation_deg_x - last_rotation_deg_x) > 0.01) or \
       (last_rotation_deg_y is None or abs(rotation_deg_y - last_rotation_deg_y) > 0.01) or \
       (last_arm_rotation_x is None or abs(arm_rotation_x - last_arm_rotation_x) > 0.01) or \
       (last_arm_rotation_z is None or abs(arm_rotation_z - last_arm_rotation_z) > 0.01) or \
       (last_focus_rotation_z is None or abs(focus_rotation_z - last_focus_rotation_z) > 0.01):
        send_rotation(rotation_deg_x, rotation_deg_y, arm_rotation_x, arm_rotation_z, focus_rotation_z)
        last_rotation_deg_x = rotation_deg_x
        last_rotation_deg_y = rotation_deg_y
        last_arm_rotation_x = arm_rotation_x
        last_arm_rotation_z = arm_rotation_z
        last_focus_rotation_z = focus_rotation_z

def gamepad_input_timer():
    handle_gamepad_input()
    return 1/60  # 1/60秒後に再度呼び出し

# タイマーを使って定期的にゲームパッドの入力を処理
bpy.app.timers.register(gamepad_input_timer, first_interval=1/60)
