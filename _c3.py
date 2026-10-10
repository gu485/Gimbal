import io
p = r"Task\component\comm_task.cpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
d = 0
for i,l in enumerate(lines,1):
    d += l.count("{") - l.count("}")
    if i in (30,40,50,60,70,80,90,100,110,120):
        print(i, d, repr(l[:60]))
