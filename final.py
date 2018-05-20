import evdev
import urllib.request
import json
import vertexTest
from collections import defaultdict
from heapq import *
from espeak import espeak
from time import sleep
import serial

lastangle=0


def getstrfromser():
    tmpstr=""
    inputstr=ser.read(1).decode()

    while(1):
        if(inputstr=='@'):
            break
    inputstr=ser.read(1).decode()

    while(inputstr!='@'):
        tmpstr=tmpstr+inputstr
        inputstr=ser.read(1).decode()
    return tmpstr


def arduino_getcompassang():
    ser.write(("@getr@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return int(tmp[1:])

def arduino_getcalcang():
    ser.write(("@getv@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return int(tmp[1:])

def arduino_setinitang(arg_ang):
    ser.write(("@seta"+str(arg_ang)+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return 

def arduino_setnorthat(arg_ang):
    ser.write(("@setn"+str(arg_ang)+"@").encode())
    tmp=getstrfromser()
    #print("********"+tmp)
    return

def arduino_turn(left,ang):
    if(left==1):
        ser.write(("@turl"+str(int(ang))+"@").encode())
    else:
        ser.write(("@turr"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_adjusttoang(ang):
    ser.write(("@adja"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_getx():
    ser.write(("@getx@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return int(tmp[1:])

def arduino_gety():
    ser.write(("@gety@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return int(tmp[1:])

def arduino_setx(ang):
    ser.write(("@setx"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_sety(ang):
    ser.write(("@sety"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_go():
    ser.write(("@go@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_stop():
    ser.write(("@stop@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_settargetx(ang):
    ser.write(("@tarx"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def arduino_settargety(ang):
    ser.write(("@tary"+str(int(ang))+"@").encode())
    tmp=getstrfromser()
    #print(tmp)
    return

def floattostr(number,keepdot):
    tmpstr=str(number)
    if(keepdot==False):
        n=tmpstr.find('.')
        if(i>0):
            return(tmpstr[0:n])
    return(tmpstr)


def getdiff(ang1,ang2):
    diff=ang1-ang2;
    if(diff>180):
        return 360-diff
    elif(diff<-180):
        return 360+diff
    elif(diff<0):
        return 0-diff
    else:
        return diff

def getinput():
    ret=""
    for event in dev.read_loop():
        if (event.value==1):
            if(event.code==96):
                break
            elif(event.code==79):
                ret=ret+"1"
            elif(event.code==80):
                ret=ret+"2"
            elif(event.code==81):
                ret=ret+"3"
            elif(event.code==75):
                ret=ret+"4"
            elif(event.code==76):
                ret=ret+"5"
            elif(event.code==77):
                ret=ret+"6"
            elif(event.code==71):
                ret=ret+"7"
            elif(event.code==72):
                ret=ret+"8"
            elif(event.code==73):
                ret=ret+"9"
            elif(event.code==82):
                ret=ret+"0"
            elif(event.code==14):
                if(len(ret)>0):
                    ret=ret[:-1]
    return ret


def dijkstra(edges, f, t):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))

    q, seen = [(0,f,[])], set()
    while q:
        #v1是f
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = list( path)
            path.append(v1)
            if v1 == t:
                return (cost, path)
            for c, v2 in g.get(v1, ()):
                if v2 not in seen:
                    heappush(q, (cost+c, v2, path))

    return float("inf")


#building=input("please input the building\n")
#level=input("please input the level\n")

building="COM1"
level="2"

dev=evdev.InputDevice('/dev/input/event0')
ser = serial.Serial('/dev/ttyS0', 9600, timeout=2)
if(ser.isOpen()==False):
    ser.open()

sleep(1)

while(True):
    

    espeak.synth("please enter the number of the building, press enter to end")
    print("please enter the number of the building, press enter to end")
    
    building=getinput()
    print("building "+building)
    
    espeak.synth("please enter the number of the level, press enter to end")
    print("please enter the number of the level, press enter to end")
    
    level=getinput()
    print("level "+level)

    f = urllib.request.urlopen("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building="+building+"&Level="+level)
    ret=f.read().decode('utf-8')
    jsonData = json.loads(ret)
    if(len(jsonData["map"])>0):
        break
    else:
        espeak.synth("no data for building "+building+" level "+level+", please re-enter a new one")
        print("no data for building "+building+" level "+level+", please re-enter a new one")
        

print("northAt:"+jsonData["info"]["northAt"])
arduino_setnorthat(int(jsonData["info"]["northAt"]))

print("node num:"+str(len(jsonData["map"])))

#http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=1&Level=2

filename="total.txt"
f=open(filename,"wb")
f.write(str.encode(ret))
f.close()
maplist,northat=vertexTest.loadnodesfromfile("total.txt")

matrix=dict()

#这部分是 计算距离
for i in range(0,len(maplist)):
    tmplinkto=maplist[i]["linkto"]
    id1=str(maplist[i]["id"])
    tmplinklist=tmplinkto.split(",")
    tmpmap=dict()
    for j in range(0,len(tmplinklist)):
        id2=tmplinklist[j]
        x1=maplist[i]["x"]
        y1=maplist[i]["y"]
        x2=maplist[int(id2)-1]["x"]
        y2=maplist[int(id2)-1]["y"]
        tmpmap[id2]=vertexTest.calcdist(x1,x2,y1,y2)
    matrix[id1]=tmpmap

angmatrix=dict()

#这部分是计算角度
for i in range(0,len(maplist)):
    tmplinkto=maplist[i]["linkto"]
    id1=str(maplist[i]["id"])
    tmplinklist=tmplinkto.split(",")
    tmpmap=dict()
    for j in range(0,len(tmplinklist)):
        id2=tmplinklist[j]
        x1=maplist[i]["x"]
        y1=maplist[i]["y"]
        x2=maplist[int(id2)-1]["x"]
        y2=maplist[int(id2)-1]["y"]
        tmpmap[id2]=vertexTest.calcang(x1,x2,y1,y2,northat)
    angmatrix[id1]=tmpmap

newedges=[]

for i in matrix:
    for j in matrix[i]:
        newedges.append((i,j,matrix[i][j]))

print(str(newedges))


#接受起点
while(True):
    espeak.synth("please enter the id of starting point, press enter to end")
    print("please enter the id of starting point, press enter to end")
    
    node1=getinput()

    if node1 not in matrix:
        espeak.synth("invalid, please re-enter")
        print("invalid, please re-enter")
    else:
        break


espeak.synth("starting point is "+node1)
print("starting point is "+node1)

#接受终点
while(True):
    espeak.synth("please enter the id of destnation, press enter to end")
    print("please enter the id of destnation, press enter to end")
    
    node2=getinput()
    if node2 not in matrix:
        espeak.synth("invalid, please re-enter")
        print("invalid, please re-enter")
    else:
        break
            
espeak.synth("destnation is "+node2)
print("destnation is "+node2)

#用dijkstra 来规划出最短路径
#这里面用heap来优化了dijkstra
cost,path=dijkstra(newedges,node1,node2)
print("total cost:"+str(cost))
print("path:"+str(path))

espeak.synth("total cost is "+str(cost/100)+" meter")

sleep(0.5)

espeak.synth("please put the car on the ground, when done, press enter")
print("please put the car on the ground")

for event in dev.read_loop():
    if (event.value==1):
        if(event.code==96):
             break

"""init_ang=arduino_getcompassang()



espeak.synth("time to turn, press enter to continue")
print("time to turn")

for event in dev.read_loop():
    if (event.value==1):
        if(event.code==96):
             break
            
arduino_setinitang(init_ang)
arduino_adjusttoang(init_ang)"""

espeak.synth("please enter heading degree, press enter to end")
print("please enter heading degree, press enter to end")
            
heading=getinput()
arduino_setinitang(heading)


espeak.synth("starting navigation")
print("starting navigation")

arduino_setx(maplist[int(path[0])-1]["x"])
arduino_sety(maplist[int(path[0])-1]["y"])
nowang=0

#这边路径因为已经用dijkstra规划好了 所以就是在path中一步步走
#每0.5s重新校正一次位置和角度
for i in range(0,len(path)-1):
    espeak.synth("guiding you to place "+str(path[i+1]))
    print("guiding you to place "+str(path[i+1])+" x:"+str(maplist[int(path[i+1])-1]["x"])+" y:"+str(maplist[int(path[i+1])-1]["y"]))

    lastdistance=268435455  #0x0fffffff
    arduino_settargetx(maplist[int(path[i+1])-1]["x"])
    arduino_settargety(maplist[int(path[i+1])-1]["y"])

    x=arduino_getx()
    y=arduino_gety()
    heading=arduino_getcalcang()
    
    x1=int(x)
    y1=int(y)
    x2=maplist[int(path[i+1])-1]["x"]
    y2=maplist[int(path[i+1])-1]["y"]
    ang=vertexTest.calcang(x1,x2,y1,y2,northat)
    arduino_adjusttoang(ang)
    print("arduino_adjusttoang:"+str(ang))
    sleep(0.5)
    
    while(True):
        """espeak.synth("do you want to enter your location? Press 1 for yes and 2 for reached there")
        print("do you want to enter your location? Press 1 for yes and 2 for reached there")
        for event in dev.read_loop():
            if (event.value==1):
                if(event.code==96):
                    break
                if(event.code==79):
                    simloac=True
                elif(event.code==80):
                    simloac=False"""
        simloac=True
        if(simloac):
            
            #espeak.synth("please enter coordinate x , press enter to end")
            #print("please enter coordinate x , press enter to end")
            x=arduino_getx()
                        
            #espeak.synth("please enter coordinate y , press enter to end")
            #print("please enter coordinate y , press enter to end")
            
            y=arduino_gety()

            #espeak.synth("please enter heading degree, press enter to end")
            #print("please enter heading degree, press enter to end")
            
            heading=arduino_getcalcang()
            
            x1=int(x)
            y1=int(y)
            x2=maplist[int(path[i+1])-1]["x"]
            y2=maplist[int(path[i+1])-1]["y"]
            #print("x:"+str(x1)+" y:"+str(y1)+" heading:"+str(heading))
            #print("x:"+str(x2)+" y:"+str(y2))
            ang=vertexTest.calcang(x1,x2,y1,y2,northat)

            #if(getdiff(ang,heading)>5):
                #arduino_stop()
                #sleep(0.2)
                #arduino_adjusttoang(ang)
                #sleep(0.2)
                #arduino_go()
            
            arduino_go()
            
            
            left,angtoturn=vertexTest.calcangtoturn(int(heading),ang)

            distance=vertexTest.calcdist(x1,x2,y1,y2)
            print("x:"+str(x)+" y:"+str(y)+" heading:"+str(heading))

            #print("left="+str(left)+" ang="+str(ang)+" angtoturn="+str(angtoturn)+" dist="+str(distance))
            #if(distance>lastdistance or distance<10):
            if((distance>lastdistance and distance-lastdistance>=5) or distance<10):
                arduino_stop()
                break
            elif(lastdistance>distance):
                lastdistance=distance

            #dstang=angmatrix[path[i]][path[i+1]]
            dstang=ang
            nowang=int(heading)
            #nowang=getang()
            
            #print("nowang is "+str(nowang));
            
            """while(getdiff(nowang,dstang)>5):
                left,angtoturn=vertexTest.calcangtoturn(nowang,dstang)
                #print("turn left "+str(left)+" angtoturn:"+str(angtoturn))
                if(left):
                    espeak.synth("turn left for "+floattostr(angtoturn,False)+ " degree")
                    print("turn left for "+floattostr(angtoturn,False)+ " degree")
                else:
                    espeak.synth("turn right for "+floattostr(angtoturn,False)+ " degree")
                    print("turn right for "+floattostr(angtoturn,False)+ " degree")
                
                if(left):
                    #turn(1,angtoturn)
                else:
                    #turn(0,angtoturn)
                sleep(0.5)
                #nowang=getang()
                nowang=dstang
            lastangle=nowang"""

            #espeak.synth("please go straight for "+floattostr(matrix[path[i]][path[i+1]]/100,False)+" meters")
            #espeak.synth("please go straight for "+floattostr(distance/100,False)+" meters")
            #espeak.synth("when you reach there, please press enter")
            """print("please go straight for "+floattostr(distance/100,False)+" meters")
            print("when you reach there, please press enter")
            
            #go(matrix[path[i]][path[i+1]])
            for event in dev.read_loop():
                if (event.value==1):
                    if(event.code==96):
                        break
                sleep(0.01)"""
            sleep(0.2)
        else:
            arduino_stop()
            sleep(0.2)
            break;
        
espeak.synth("you have reach the destnation, navigation is over, thanks for using")


