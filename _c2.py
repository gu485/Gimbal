import io
p = r"Task\component\comm_task.cpp"
lines = io.open(p, encoding="utf-8").read().split("\n")
d = 0
for i,l in enumerate(lines,1):
    d += l.count("{") - l.count("}")
    if i>=85 and d<0:
        print("NEG at line",i,repr(l)); break
print("depth at 80/150/200/240/255:", )
for k in (80,150,200,240,len(lines)):
    dd=0
    for l in lines[:k]: dd += l.count("{")-l.count("}")
    print(" after line",k,"depth",dd)
