import json
import math

def loadnodesfromfile(file):
    f=open(file,"rb")
    ret=f.read().decode('utf-8')
    f.close()
    tmplist=[]
    
    jsonData = json.loads(ret)
    northat=int(jsonData["info"]["northAt"])
    for i in range(0,len(jsonData["map"])):
        tmpdata={}
        tmpdata["id"]=jsonData["map"][i]["nodeId"]
        tmpdata["x"]=int(jsonData["map"][i]["x"])
        tmpdata["y"]=int(jsonData["map"][i]["y"])
        tmpdata["nodeName"]=jsonData["map"][i]["nodeName"]
        tmpedge=jsonData["map"][i]["linkTo"]
        tmpedge=tmpedge.replace(" ","")
        tmpdata["linkto"]=tmpedge
        tmplist.append(tmpdata)
    return tmplist,northat

def calcdist(x1,x2,y1,y2):
    return (((x1-x2)**2+(y1-y2)**2)**0.5)

def calcang(x1,x2,y1,y2,northat):
    rawang=calcang_raw(x1,x2,y1,y2)
    rawang=rawang+360-northat
    if(rawang>=360):
        rawang=rawang-360
    return rawang

def calcang_raw(x1,x2,y1,y2):
    if(x1==x2 and y1==y2):
        return 0
    if(y2-y1==0):
        if(x2-x1>=0):
            return 90
        else:
            return 270
    else:
        rad=math.atan((x2-x1)/(y2-y1))
        deg=rad*180/math.pi
        if(deg<0):
            if(y2-y1>0):
                deg=deg+360
            else:
                deg=180+deg
        else:
            if(y2-y1<0):
                deg=deg+180
    return deg

def calcangtoturn(cur,dst):
    turnleft=False
    angtoturn=dst-cur
    if(angtoturn>0):
        if(angtoturn>180):
            angtoturn=360-angtoturn
            turnleft=True
        else:
            turnleft=False
    else:
        if(angtoturn<-180):
            angtoturn=360+angtoturn
            turnleft=False
        else:
            turnleft=True
            angtoturn=0-angtoturn
    return turnleft,angtoturn

"""
maplist=loadnodesfromfile("z:\\2.txt")

for i in range(0,len(maplist)):
    print(maplist[i]["nodeName"]+"("+str(maplist[i]["x"])+","+str(maplist[i]["y"])+")")

node1=int(input("please input the id of nod1:\n"))
node2=int(input("please input the id of nod2:\n"))

x1=maplist[node1-1]["x"]
x2=maplist[node2-1]["x"]
y1=maplist[node1-1]["y"]
y2=maplist[node2-1]["y"]

print("the distance between them is "+str(((x1-x2)**2+(y1-y2)**2)**0.5))

if(y2-y1==0):
    if(x2-x1<=0):
        print("the angle between them is 270")
    else:
        print("the angle between them is 90")
else:
    rad=math.atan((x2-x1)/(y2-y1))
    deg=rad*180/math.pi
    if(deg<0):
        deg=180+deg
    print("the angle between them is "+str(deg))
"""


