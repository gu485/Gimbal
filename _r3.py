import io
p = r"Task\component\comm_task.cpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
# 从后往前删,避免行号偏移。1-based -> 0-based
del lines[228:238]   # 229-238 pitch 镜像赋值块
del lines[172:182]   # 173-182 pitch 发帧块(含前导空行)
del lines[56:70]     # 57-70 pitch 调试变量块
io.open(p,"w",encoding="utf-8").write("\n".join(lines))
print("ok")
