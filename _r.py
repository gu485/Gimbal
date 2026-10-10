import io
p = r"RobotModules\component\gimbal.cpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
#del lines[231:292]  # 0-based: 行232..292(292为空行)
del lines[231:292]
io.open(p,"w",encoding="utf-8").write("\n".join(lines))
print("removed")
