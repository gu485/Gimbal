import io
p = r"RobotModules\component\gimbal.hpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
#删 54行(pitch_pid_)、73-84(状态块)  0-based: 53, 72..84
del lines[72:84]
del lines[53]
io.open(p,"w",encoding="utf-8").write("\n".join(lines))
print("ok")
