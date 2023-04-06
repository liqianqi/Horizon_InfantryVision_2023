import cv2
import math
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
# 创建3d绘图区域
ax = plt.axes(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

pose_x = []
pose_y = []
pose_z = []
points_3D = [[-268.7, 0, 0], [0, 268.7, 0], [268.7, 0, 0], [0, -268.7, 0]]
points_3D = np.array(points_3D, dtype="double")
camera_matrix = [[1.661923585022047e+03, 0, 9.566273843667578e+02], [0, 1.661966349223287e+03, 5.328345956293313e+02], [0, 0, 1]]
camera_matrix = np.array(camera_matrix, dtype="double")
dist = np.zeros((4, 1), dtype="double")
P_old = []
A, B, C, D = 0, 0, 0, 0
counter = 0


def rotate_by_z(x, y, theta_z):
    outx = math.cos(theta_z) * x - math.sin(theta_z) * y
    outy = math.sin(theta_z) * x + math.cos(theta_z) * y
    return outx, outy


def rotate_by_x(y, z, theta_x):
    outy = math.cos(theta_x) * y - math.sin(theta_x) * z
    outz = math.sin(theta_x) * y + math.cos(theta_x) * z
    return outy, outz


def rotate_by_y(z, x, theta_y):
    outz = math.cos(theta_y) * z - math.sin(theta_y) * x
    outx = math.sin(theta_y) * z + math.cos(theta_y) * x
    return outz, outx


def get_euler_angle(rotation_vector):
    # 旋转顺序是z,y,x，对于相机来说就是滚转，偏航，俯仰
    # calculate rotation angles
    theta = cv2.norm(rotation_vector, cv2.NORM_L2)

    # transformed to quaterniond
    w = math.cos(theta / 2)
    x = math.sin(theta / 2) * rotation_vector[0][0] / theta
    y = math.sin(theta / 2) * rotation_vector[1][0] / theta
    z = math.sin(theta / 2) * rotation_vector[2][0] / theta

    ysqr = y * y
    # pitch (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + ysqr)
    pitch = math.atan2(t0, t1)

    # yaw (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
    if t2 < -1.0:
        t2 = -1.0
    yaw = math.asin(t2)

    # roll (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (ysqr + z * z)
    roll = math.atan2(t3, t4)

    return roll, pitch, yaw


def approx(con, side_size):  # con为预先得到的最大轮廓
    num = 0.0005
    # 初始化时不需要太小，因为四边形所需的值并不很小
    ep = num * cv2.arcLength(con, True)
    con = cv2.approxPolyDP(con, ep, True)
    while True:
        if len(con) <= side_size:  # 防止程序崩溃设置的<=4
            break
        else:
            num = num * 1.5
            ep = num * cv2.arcLength(con, True)
            con = cv2.approxPolyDP(con, ep, True)
            continue

    return con


def distance(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


vc = cv2.VideoCapture('test.mp4')
first_frame = True
if vc.isOpened():
    op = True
else:
    op = False

while op:
    ret, frame = vc.read()
    if frame is None:
        break
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    ret1, binary = cv2.threshold(s, 75, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=lambda cnts: cv2.contourArea(cnts, True), reverse=False)
    # cv2.drawContours(frame, contours, 0, (255, 0, 255), 2)
    cnt = approx(contours[0], 4)
    if first_frame:
        A_old = cnt[0][0]
        B_old = cnt[3][0]
        C_old = cnt[2][0]
        D_old = cnt[1][0]
        P_old = [A, B, C, D] = [A_old, B_old, C_old, D_old]
        first_frame = False
    else:
        for P in cnt:
            error = []
            for P_ in P_old:
                error.append(distance(P[0], P_))
            index = error.index(min(error))
            if index == 0:
                A = P[0]
            elif index == 1:
                B = P[0]
            elif index == 2:
                C = P[0]
            else:
                D = P[0]
            P_old = [A, B, C, D]

    P_ = [A, B, C, D]
    points_2D = np.array(P_, dtype="double")
    ret, rvec, tvec = cv2.solvePnP(points_3D, points_2D, camera_matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    (Z_end, jacobian_z) = cv2.projectPoints(np.array([(0.0, 0.0, 500.0)]), rvec, tvec, camera_matrix, dist)
    (X_end, jacobian_x) = cv2.projectPoints(np.array([(500.0, 0.0, 0.0)]), rvec, tvec, camera_matrix, dist)
    (Y_end, jacobian_y) = cv2.projectPoints(np.array([(0.0, 500.0, 0.0)]), rvec, tvec, camera_matrix, dist)
    (O, jacobian_o) = cv2.projectPoints(np.array([(0.0, 0.0, 0.0)]), rvec, tvec, camera_matrix, dist)
    Z_end = (int(Z_end[0][0][0]), int(Z_end[0][0][1]))
    X_end = (int(X_end[0][0][0]), int(X_end[0][0][1]))
    Y_end = (int(Y_end[0][0][0]), int(Y_end[0][0][1]))
    O_ = (int(O[0][0][0]), int(O[0][0][1]))
    cv2.line(frame, O_, X_end, (50, 255, 50), 10)
    cv2.line(frame, O_, Y_end, (50, 50, 255), 10)
    cv2.line(frame, O_, Z_end, (255, 50, 50), 10)
    cv2.putText(frame, "X", X_end, cv2.FONT_HERSHEY_SIMPLEX, 3, (5, 150, 5), 10)
    cv2.putText(frame, "Y", Y_end, cv2.FONT_HERSHEY_SIMPLEX, 3, (5, 5, 150), 10)
    cv2.putText(frame, "Z", Z_end, cv2.FONT_HERSHEY_SIMPLEX, 3, (150, 5, 5), 10)

    cv2.circle(frame, A, 3, (255, 255, 255), 10)
    cv2.circle(frame, B, 3, (255, 255, 255), 10)
    cv2.circle(frame, C, 3, (255, 255, 255), 10)
    cv2.circle(frame, D, 3, (255, 255, 255), 10)

    cv2.putText(frame, "A", A + [15, 15], cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 5)
    cv2.putText(frame, "B", B + [15, 15], cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 5)
    cv2.putText(frame, "C", C + [15, 15], cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 5)
    cv2.putText(frame, "D", D + [15, 15], cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 5)

    xt, yt, zt = tvec
    xr, yr, zr = rvec
    rvec_str = 'rotation_vector: ({0}, {1}, {2})'
    rvec_str = rvec_str.format(float(xr), float(yr), float(zr))
    tvec_str = 'translation_vector: ({0}, {1}, {2})'
    tvec_str = tvec_str.format(float(xt), float(yt), float(zt))

    cv2.putText(frame, tvec_str, [30, 30], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
    cv2.putText(frame, rvec_str, [30, 80], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)

    roll, pitch, yaw = get_euler_angle(rvec)
    # print(roll, pitch, yaw)

    Xc, Yc, Zc = tvec
    Xc, Yc = rotate_by_z(Xc, Yc, -roll)
    Zc, Xc = rotate_by_y(Zc, Xc, -yaw)
    Yc, Zc = rotate_by_x(Yc, Zc, -pitch)

    pose_x.append(-Xc)
    pose_y.append(-Yc)
    pose_z.append(-Zc)

    position_str = 'position: ({0}, {1}, {2})'
    position_str = position_str.format(float(-Xc), float(-Yc), float(-Zc))
    cv2.putText(frame, position_str, [30, 130], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)

    counter = counter + 1

    cv2.imshow("res", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

pose_x = np.array(pose_x)
pose_y = np.array(pose_y)
pose_z = np.array(pose_z)

ax.plot3D(pose_x, pose_y, pose_z, linewidth=0.4)
ax.set_title('Trace')
plt.show()
