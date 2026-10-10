import io
p = r"RobotModules\component\gimbal.cpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
d = 0
for i, l in enumerate(lines, 1):
    d += l.count("{") - l.count("}")
    if i >= 60 and d < 0:
        print("NEG at", i, repr(l)); break
print("final depth:", d)
