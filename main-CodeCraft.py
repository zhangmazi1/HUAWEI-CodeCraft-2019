import copy
import numpy as np

cross_txt = np.loadtxt('cross.txt', dtype=str, comments='#', delimiter=None)
car_txt   = np.loadtxt('car.txt',   dtype=str, comments='#', delimiter=None)
road_txt  = np.loadtxt('road.txt',  dtype=str, comments='#', delimiter=None)

for i in range(cross_txt.shape[0]):
    for j in range(cross_txt.shape[1]):
        cross_txt[i][j] = cross_txt[i][j].replace(',', '').replace('(', '').replace(')', '')
for i in range(car_txt.shape[0]):
    for j in range(car_txt.shape[1]):
        car_txt[i][j]   =   car_txt[i][j].replace(',', '').replace('(', '').replace(')', '')
for i in range(road_txt.shape[0]):
    for j in range(road_txt.shape[1]):
        road_txt[i][j]  =  road_txt[i][j].replace(',', '').replace('(', '').replace(')', '')

crosstxt = np.array(cross_txt, dtype='int')
cartxt   = np.array(car_txt  , dtype='int')
roadtxt  = np.array(road_txt , dtype='int')

path_result = np.load('path.npy')                #读取有其他算法输出得到的路径信息，我们将数据保存为.npy文件
path_ = []
for one_path in path_result:
    b = []
    for i in range(len(one_path) - 2):
        a = (one_path[i + 1], one_path[i + 2])
        b.append(a)
    path_.append(one_path[0])
    path_.append(b)
#print(path_)


def search_nn(nodenum):                 #搜索该节点能够连接到的节点，并将各节点返回为一个list
    a=[]
    for i in range(len(roadtxt)):
        if roadtxt[i][4] == nodenum and roadtxt[i][6]==1:
            a.append(roadtxt[i][5])
        if roadtxt[i][5] == nodenum:
            a.append(roadtxt[i][4])
    return a


def serach_roadlimitspeed(start, end):  #根据起始节点确定路的信息，道路限速
    for i in range(len(roadtxt)):
        if ([start,end] ==list(roadtxt[i,4:6]) or [end,start] ==list(roadtxt[i,4:6])):
            return roadtxt[i][2]


def serach_roadid(start, end):          #根据起始节点确定路的信息，道路编号
    for i in range(len(roadtxt)):
        if ([start,end] ==list(roadtxt[i,4:6]) or [end,start] ==list(roadtxt[i,4:6])):
            return roadtxt[i][0]


def search_cross(crossid):              #根据路口节点id 返回 一行节点向量
    for i in range(crosstxt.shape[0]):
        if crosstxt[i][0] == crossid:
            return list(crosstxt[i])


def search_car(car_id):                           #根据车辆节点id 返回 一行汽车信息向量
    for i in range(cartxt.shape[0]):
        if cartxt[i][0] == car_id:
            return list(cartxt[i])


def next_direction_decide(car_object):            #输入为汽车对象， 确定车辆前方是否是终点，或者前方转向方向
    if car_object !=-1:
        current_road = car_object.path[car_object.path_num]

        if car_object.path_num == len(car_object.path)-1:
            return 'END'
        else:
            next_road = car_object.path[car_object.path_num+1]
            current_roadid=serach_roadid(*current_road)
            next_roadid = serach_roadid(*next_road)
            A=search_cross(current_road[1]).index(current_roadid)
            B=search_cross(current_road[1]).index(next_roadid)
            C=(A-B+4)%4
            if C==1:
                return 'right'
            if C==2:
                return 'direct'
            if C==3:
                return 'left'
    else:
        return 'bugbug'

def find_priority_car(road_matrix,car_onroad_dict1):                #输入为某条道路矩阵， 判断某条道路上的车辆优先级，返回优先级最高的车辆
    death_channel = []
    for i in reversed(range(road_matrix.shape[1])):
        for j in range(road_matrix.shape[0]):
            if j in death_channel:
                continue
            if road_matrix[j][i] != 0:
                car_obj=car_onroad_dict1[road_matrix[j][i]]
                if car_obj.state!=2:
                    if not  (min(car_obj.limitspeed,serach_roadlimitspeed(car_obj.location[0],car_obj.location[1])) <= road_matrix.shape[1]-1-car_obj.location[3] or((car_obj.path_num !=(len(car_obj.path)-1)) and road_matrix.shape[1]-1-car_obj.location[3] >= serach_roadlimitspeed(*car_obj.path[car_obj.path_num+1]))) :
                        return car_obj
                    else:
                        death_channel.append(j)
                else:
                    death_channel.append(j)
    return -1


def go_or_not_judge(car_a,car_b,car_c,car_d,aa):
    if aa==0 and car_a!=-1:
        
        if next_direction_decide(car_a)=='direct':
            return True
        if next_direction_decide(car_a)=='left' and next_direction_decide(car_d)!='direct':
            return True
        if next_direction_decide(car_a)=='right' and next_direction_decide(car_b)!='direct' and next_direction_decide(car_c)!='left':
            return True
        
    elif aa==1 and car_b!=-1:
        if next_direction_decide(car_b)=='direct':
            return True
        if next_direction_decide(car_b)=='left' and next_direction_decide(car_a)!='direct':
            return True
        if next_direction_decide(car_b)=='right' and next_direction_decide(car_c)!='direct' and next_direction_decide(car_d)!='left':
            return True
    elif aa==2 and car_c!=-1:
        if next_direction_decide(car_c)=='direct':
            return True
        if next_direction_decide(car_c)=='left' and next_direction_decide(car_b)!='direct':
            return True
        if next_direction_decide(car_c)=='right' and next_direction_decide(car_d)!='direct' and next_direction_decide(car_a)!='left':
            return True
    elif aa==3 and car_d!=-1:
        if next_direction_decide(car_d)=='direct':
            return True
        if next_direction_decide(car_d)=='left' and next_direction_decide(car_c)!='direct':
            return True
        if next_direction_decide(car_d)=='right' and next_direction_decide(car_a)!='direct' and next_direction_decide(car_b)!='left':
            return True
    else:
        return False


def road_matrix_order(cross_id):
    pp = search_nn(cross_id)
    d = {}
    dd = {}
    ddd = {}
    for i in range(len(pp)):
        d.update({search_cross(cross_id).index(serach_roadid(pp[i],cross_id)):serach_roadid(pp[i],cross_id)})
        dd.update({serach_roadid(pp[i],cross_id):search_cross(cross_id).index(serach_roadid(pp[i],cross_id))})
        ddd.update({search_cross(cross_id).index(serach_roadid(pp[i],cross_id)):(pp[i],cross_id)})
    return d,dd,ddd


def add_newcar(car__object,path_list,map_object,caronroad_dict):      #输入分别为汽车对象，该车行驶路径，地图对象，已在车道上的 车id：车对象 的dict

    new_car=car__object
    new_car.path = path_list
    next_roadmatrix=map_object.road_info[path_list[0]]
    maxdistance=min(serach_roadlimitspeed(*path_list[0]),new_car.limitspeed)
    in_channelnum=999

    for i in range(next_roadmatrix.shape[0]):
        if next_roadmatrix[i][0] !=0:
            continue 
        else:
            in_channelnum = i
            break
    nnn=0
    if in_channelnum !=999:
        for j in range(maxdistance):
            if  next_roadmatrix[in_channelnum][j] ==0 :
                nnn=j
            else:
                break
        new_car.state=2
        new_car.location=[*path_list[0],in_channelnum,nnn]  #放在路，更新车的信息与地图车道信息
        next_roadmatrix[in_channelnum][nnn]=car__object.carid
        new_car.update_car()
        caronroad_dict.update({car__object.carid:new_car})
    else:
        return 'join_fail'


class car():                             #(id,from,to,speed,planTime)
    def __init__(self,cart):
        self.carid=cart[0]
        self.carfrom=cart[1]
        self.carto=cart[2]
        self.limitspeed=cart[3]
        self.planTime=cart[4]
        self.trueTime=cart[4]            #初始化时 假设能按照预定时间出发
        self.state=0                     #0代表未处理，1代表等待，2代表处理完 3代表到达目的地?
        self.location=[]
        self.path=[]
        self.path_num=0
        self.yty=0
    def update_car(self):
        self.path_num=self.path.index((self.location[0],self.location[1]))


class map_info():

    def __init__(self,crosst,roadt):
        self.cross=crosst  #(id,roadId,roadId,roadId,roadId)
        self.road =roadt   #(id,length,speed,channel,from,to,isDuplex)
        self.road_info={}
        for i in range(len(self.road)):
            c={(self.road[i][4],self.road[i][5]):np.array([[0 for y in range(self.road[i][1])] for x in range(self.road[i][3])])}
            if self.road[i][6] == 1:
                self.road_info.update({(self.road[i][5],self.road[i][4]):np.array([[0 for y in range(self.road[i][1])] for x in range(self.road[i][3])])})
            self.road_info.update(c)                   #tuple格式为索引的dict，对应的值为道路信息矩阵

    def control_car(self,car_contrilling, car_onroad_dict):

        carid=car_contrilling.carid
        print(carid)
        roadstart=car_contrilling.location[0]
        roadend=car_contrilling.location[1]
        roadchannel=car_contrilling.location[2]
        roadloc=car_contrilling.location[3]
        roadmatrix=self.road_info[(roadstart,roadend)]  #该车所在当前道路信息矩阵
        runspeed = min(car_contrilling.limitspeed,serach_roadlimitspeed(roadstart,roadend))
        rest_roadlen=roadmatrix.shape[1]-1-roadloc
        path=car_contrilling.path
        path_num=car_contrilling.path_num
#####################################################################################################################################                 
        if (runspeed <= rest_roadlen) or ( (path_num !=(len(path)-1)) and rest_roadlen >= serach_roadlimitspeed(*path[path_num+1]) ):   #如果该车不会转弯
            if runspeed >=rest_roadlen:
                runspeed=rest_roadlen
            for j in range(1,runspeed+1):

                if roadmatrix[roadchannel][roadloc+j] !=0:
                    forward_carid = roadmatrix[roadchannel][roadloc+j]

                    if  car_onroad_dict[forward_carid].state ==2 and car_contrilling.state!=2 :       #前方车辆不能走，直接停在前方车辆后面
                        car_contrilling.state=2 
                        car_contrilling.location[3]=roadloc+j-1
                        roadmatrix[roadchannel][roadloc] =0
                        roadmatrix[roadchannel][roadloc+j-1]=carid
                        break

                    if car_onroad_dict[forward_carid].state ==1:       #阻碍车状态为1，说明陷入死锁，则车不允许此类方式前进，直接终止，完全跳出循环
                        return 'death_lock'                            #死锁情况，由于此循环位于类的函数中，则用return语句中断最外层循环，返回值为'death_lock_'

                    if car_onroad_dict[forward_carid].state ==0:       #先去处理前方车辆
                        car_contrilling.state=1
                        if self.control_car(car_onroad_dict[forward_carid], car_onroad_dict) == 'death_lock':
                            return 'death_lock'
                        if roadmatrix[roadchannel][roadloc+j] !=0 and car_contrilling.state!=2:
                            car_contrilling.state=2 
                            car_contrilling.location[3]=roadloc+j-1
                            roadmatrix[roadchannel][roadloc] =0
                            roadmatrix[roadchannel][roadloc+j-1]=carid
                            break

            if  car_contrilling.state!=2:                               #前方无阻碍车辆的情况
                car_contrilling.state=2 
                car_contrilling.location[3]=roadloc+runspeed
                roadmatrix[roadchannel][roadloc] =0
                roadmatrix[roadchannel][roadloc+runspeed]=carid
##################################################################################################################
        else:
            list_traversal1=[(roadchannel-i-1,roadloc) for i in range(roadchannel)]
            list_traversal2=[(i,j) for j in range(roadloc+1,roadmatrix.shape[1]) for i in reversed(range(roadmatrix.shape[0]))]
            list_traversal=list_traversal1+list_traversal2                  #按顺序取剩余矩阵位置放回 一对坐标的tuple 的list

            for j in list_traversal:

                if roadmatrix[j[0],j[1]] !=0:
                    forward_carid = roadmatrix[j[0],j[1]]
                    forward_car = car_onroad_dict[forward_carid]
                    if forward_car.state ==1:
                        return 'death_lock'

                    elif forward_car.state ==2 and car_contrilling.state!=2:
                        if roadchannel == forward_car.location[2]:
                            car_contrilling.state=2 
                            car_contrilling.location[3]=j[1]-1
                            roadmatrix[roadchannel][roadloc] =0
                            roadmatrix[roadchannel][j[1]-1]=carid
                            break

                    elif forward_car.state ==0 and car_contrilling.state!=2:
                        car_contrilling.state=1

                        if self.control_car(forward_car,car_onroad_dict) =='death_lock':
                            return 'death_lock'
                        if (roadchannel == forward_car.location[2] and roadmatrix[j[0],j[1]] !=0 and car_contrilling.state!=2):
                            car_contrilling.state=2 
                            car_contrilling.location[3]=j[1]-1
                            roadmatrix[roadchannel][roadloc] =0
                            roadmatrix[roadchannel][j[1]-1]=carid
                            break
            if (car_contrilling.state != 2 and next_direction_decide(car_contrilling) == 'END') :   #汽车到达终点，并更新道路上的汽车dict与list
                roadmatrix[roadchannel][roadloc] = 0
                #car_onroad_dict[car_contrilling.carid].state=3
                del car_onroad_dict[car_contrilling.carid]
                print(car_contrilling.carid, allcardict[carid])
                
#################################################################################################################
            
            elif (next_direction_decide(car_contrilling) == 'direct' and car_contrilling.state!=2):                                 #汽车直行，看停在哪
                next_roadmatrix=self.road_info[path[path_num+1]]
                maxdistance=min(serach_roadlimitspeed(*path[path_num+1]),car_contrilling.limitspeed)-rest_roadlen
                in_channelnum=999
                for i in range(next_roadmatrix.shape[0]):
                    if next_roadmatrix[i][0] !=0:
                        if car_onroad_dict[next_roadmatrix[i][0]].state == 0:
                            if self.control_car(car_onroad_dict[next_roadmatrix[i][0]], car_onroad_dict)=='death_lock':
                                return 'death_lock'
                            if next_roadmatrix[i][0] ==0:
                                in_channelnum = i
                                break
                        if car_onroad_dict[next_roadmatrix[i][0]].state == 1:
                            return 'death_lock'

                    else:
                        in_channelnum = i
                        break
                if in_channelnum !=999:
                    for j in range(maxdistance):
                        if next_roadmatrix[in_channelnum][j] !=0 :
                            if  car_onroad_dict[next_roadmatrix[in_channelnum][j]].state == 0:
                                if self.control_car(car_onroad_dict[next_roadmatrix[in_channelnum][j]],car_onroad_dict)=='death_lock':
                                    return 'death_lock'
                                if next_roadmatrix[in_channelnum][j] !=0 and car_contrilling.state!=2:
                                    car_contrilling.state=2
                                    car_contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                    roadmatrix[roadchannel][roadloc] =0
                                    next_roadmatrix[in_channelnum][j]=carid
                                    car_contrilling.update_car()
                                    break
                            elif  car_onroad_dict[next_roadmatrix[in_channelnum][j]].state == 1:
                                return 'death_lock'
                            elif  car_onroad_dict[next_roadmatrix[in_channelnum][j]].state == 2 and car_contrilling.state!=2:

                                car_contrilling.state=2
                                car_contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                roadmatrix[roadchannel][roadloc] =0
                                next_roadmatrix[in_channelnum][j-1]=carid
                                car_contrilling.update_car()
                                break
                    if  car_contrilling.state!=2:
                        car_contrilling.state=2
                        car_contrilling.location=[*path[path_num+1],in_channelnum,maxdistance-1]  #放在前面一辆车后面，更新车的信息与车道信息
                        roadmatrix[roadchannel][roadloc] =0
                        next_roadmatrix[in_channelnum][maxdistance-1]=carid
                        car_contrilling.update_car()

                elif car_contrilling.state!=2:
                    car_contrilling.state=2 
                    car_contrilling.location[3]=roadloc+rest_roadlen
                    roadmatrix[roadchannel][roadloc] =0
                    roadmatrix[roadchannel][roadloc+rest_roadlen]=carid
##########################################################################################

            elif (next_direction_decide(car_contrilling)== 'left' or next_direction_decide(car_contrilling)== 'right') and car_contrilling.state!=2:
                dict_1,dict_2,dict_3 =road_matrix_order(car_contrilling.location[1])
                procar_order=[-1,-1,-1,-1]
                carid_oder=[-1,-1,-1,-1]
                for i in dict_3:
                    procar_order[i-1]=find_priority_car(self.road_info[dict_3[i]],car_onroad_dict)
                    carid_oder[i-1]=dict_1[i]
                b=carid_oder[:]
                b.sort()
                c=[carid_oder.index(i) for i in b]
                crossid_cur=car_contrilling.location[1]

                while(car_contrilling.state!=2):
                    while(go_or_not_judge(*procar_order,c[0])):
                        car__contrilling=procar_order[c[0]]
                        roadloc=car__contrilling.location[3]
                        roadchannel=car__contrilling.location[2]
                        path= car__contrilling.path
                        path_num=car__contrilling.path_num
                        rest_roadlen = self.road_info[path[path_num]].shape[1] - 1 - car__contrilling.location[3]
                        roadmatrix1 = self.road_info[(car__contrilling.location[0], car__contrilling.location[1])]
                        next_roadmatrix1=self.road_info[path[path_num+1]]
                        maxdistance=min(serach_roadlimitspeed(*path[path_num+1]),car__contrilling.limitspeed)-rest_roadlen
                        in_channelnum=999
                        carid=car__contrilling.carid
                        print('while_1', car_contrilling.carid, car__contrilling.carid)
                        for i in range(next_roadmatrix1.shape[0]):
                            if next_roadmatrix1[i][0] !=0:
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 0:
                                    if self.control_car(car_onroad_dict[next_roadmatrix1[i][0]],car_onroad_dict)=='death_lock':
                                        return 'death_lock'
                                    if next_roadmatrix1[i][0] ==0:
                                        in_channelnum = i
                                        break
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 1:
                                    return 'death_lock'

                            else:
                                in_channelnum = i
                                break
                        if in_channelnum !=999:
                            for j in range(maxdistance):
                                if next_roadmatrix1[in_channelnum][j] !=0 :
                                    if  car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state ==0:
                                        if self.control_car(car_onroad_dict[next_roadmatrix1[in_channelnum][j]],car_onroad_dict)=='death_lock':
                                            return 'death_lock'
                                        if next_roadmatrix1[in_channelnum][j] !=0 and car__contrilling.state!=2:
                                            car__contrilling.state=2
                                            car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                            roadmatrix1[roadchannel][roadloc] =0
                                            next_roadmatrix1[in_channelnum][j]=carid
                                            car__contrilling.update_car()
                                            break
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 1:
                                        return 'death_lock'
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 2 and car_contrilling.state!=2:

                                        car__contrilling.state=2
                                        car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                        roadmatrix1[roadchannel][roadloc] =0
                                        next_roadmatrix1[in_channelnum][j]=carid
                                        car__contrilling.update_car()
                                        break
                            if car__contrilling.state!=2:
                                car__contrilling.state=2
                                car__contrilling.location=[*path[path_num+1],in_channelnum,maxdistance-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                roadmatrix1[roadchannel][roadloc] =0
                                next_roadmatrix1[in_channelnum][maxdistance-1]=carid
                                car__contrilling.update_car()

                        elif car__contrilling.state!=2:
                            car__contrilling.state=2
                            car__contrilling.location[3]=roadloc+rest_roadlen
                            roadmatrix1[roadchannel][roadloc] =0
                            roadmatrix1[roadchannel][roadloc+rest_roadlen]=carid

                        dict_1,dict_2,dict_3 =road_matrix_order(crossid_cur)
                        procar_order=[-1,-1,-1,-1]
                        carid_oder=[-1,-1,-1,-1]
                        for i in dict_3:
                            procar_order[i-1]=find_priority_car(self.road_info[dict_3[i]],car_onroad_dict)
                            carid_oder[i-1]=dict_1[i]
                        b=carid_oder[:]
                        b.sort()
                        c=[carid_oder.index(i) for i in b]

                    while(go_or_not_judge(*procar_order,c[1])):
                        print('while_2')
                        car__contrilling = procar_order[c[1]]
                        roadloc = car__contrilling.location[3]
                        roadchannel = car__contrilling.location[2]
                        path = car__contrilling.path
                        path_num = car__contrilling.path_num
                        rest_roadlen = self.road_info[path[path_num]].shape[1] - 1 - car__contrilling.location[3]
                        roadmatrix1 = self.road_info[(car__contrilling.location[0], car__contrilling.location[1])]
                        next_roadmatrix1=self.road_info[path[path_num+1]]
                        maxdistance=min(serach_roadlimitspeed(*path[path_num+1]),car__contrilling.limitspeed)-rest_roadlen
                        in_channelnum=999
                        carid = car__contrilling.carid
                        print(carid)
                        for i in range(next_roadmatrix1.shape[0]):
                            if next_roadmatrix1[i][0] !=0:
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 0:
                                    if self.control_car(car_onroad_dict[next_roadmatrix1[i][0]],car_onroad_dict)=='death_lock':
                                        return 'death_lock'
                                    if next_roadmatrix1[i][0] ==0:
                                        in_channelnum = i
                                        break
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 1:
                                    return 'death_lock'

                            else:
                                in_channelnum = i
                                break
                        if in_channelnum !=999:
                            for j in range(maxdistance):
                                if next_roadmatrix1[in_channelnum][j] !=0 :
                                    if  car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state ==0:
                                        if self.control_car(car_onroad_dict[next_roadmatrix1[in_channelnum][j]],car_onroad_dict)=='death_lock':
                                            return 'death_lock'
                                        if next_roadmatrix1[in_channelnum][j] !=0 and car__contrilling.state!=2:
                                            car__contrilling.state=2
                                            car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                            roadmatrix1[roadchannel][roadloc] =0
                                            next_roadmatrix1[in_channelnum][j]=carid
                                            car__contrilling.update_car()
                                            break
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 1:
                                        return 'death_lock'
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 2 and car_contrilling.state!=2:

                                        car__contrilling.state=2
                                        car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                        roadmatrix1[roadchannel][roadloc] =0
                                        next_roadmatrix1[in_channelnum][j]=carid
                                        car__contrilling.update_car()
                                        break
                            if car__contrilling.state!=2:
                                car__contrilling.state=2
                                car__contrilling.location=[*path[path_num+1],in_channelnum,maxdistance-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                roadmatrix1[roadchannel][roadloc] =0
                                next_roadmatrix1[in_channelnum][maxdistance-1]=carid
                                car__contrilling.update_car()
                        elif car__contrilling.state!=2:
                            car__contrilling.state=2
                            car__contrilling.location[3]=roadloc+rest_roadlen
                            roadmatrix1[roadchannel][roadloc] =0
                            roadmatrix1[roadchannel][roadloc+rest_roadlen]=carid
                        dict_1,dict_2,dict_3 =road_matrix_order(crossid_cur)
                        procar_order=[-1,-1,-1,-1]
                        carid_oder=[-1,-1,-1,-1]
                        for i in dict_3:
                            procar_order[i-1]=find_priority_car(self.road_info[dict_3[i]],car_onroad_dict)
                            carid_oder[i-1]=dict_1[i]
                        b=carid_oder[:]
                        b.sort()
                        c=[carid_oder.index(i) for i in b]
                    while(go_or_not_judge(*procar_order,c[2])):
                        print('while_3')
                        car__contrilling = procar_order[c[2]]
                        roadloc = car__contrilling.location[3]
                        roadchannel = car__contrilling.location[2]
                        path = car__contrilling.path
                        path_num = car__contrilling.path_num
                        rest_roadlen = self.road_info[path[path_num]].shape[1] - 1 - car__contrilling.location[3]
                        roadmatrix1 = self.road_info[(car__contrilling.location[0], car__contrilling.location[1])]
                        next_roadmatrix1=self.road_info[path[path_num+1]]
                        maxdistance=min(serach_roadlimitspeed(*path[path_num+1]),car__contrilling.limitspeed)-rest_roadlen
                        in_channelnum=999
                        carid = car__contrilling.carid
                        for i in range(next_roadmatrix1.shape[0]):
                            if next_roadmatrix1[i][0] !=0:
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 0:
                                    if self.control_car(car_onroad_dict[next_roadmatrix1[i][0]],car_onroad_dict)=='death_lock':
                                        return 'death_lock'
                                    if next_roadmatrix1[i][0] ==0:
                                        in_channelnum = i
                                        break
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 1:
                                    return 'death_lock'

                            else:
                                in_channelnum = i
                                break
                        if in_channelnum !=999:
                            for j in range(maxdistance):
                                if next_roadmatrix1[in_channelnum][j] !=0 :
                                    if  car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state ==0:
                                        if self.control_car(car_onroad_dict[next_roadmatrix1[in_channelnum][j]], car_onroad_dict)=='death_lock':
                                            return 'death_lock'
                                        if next_roadmatrix1[in_channelnum][j] !=0 and car__contrilling.state!=2:
                                            car__contrilling.state=2
                                            car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                            roadmatrix1[roadchannel][roadloc] =0
                                            next_roadmatrix1[in_channelnum][j]=carid
                                            car__contrilling.update_car()
                                            break
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 1:
                                        return 'death_lock'
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 2 and car__contrilling.state!=2:

                                        car__contrilling.state=2
                                        car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                        roadmatrix1[roadchannel][roadloc] =0
                                        next_roadmatrix1[in_channelnum][j]=carid
                                        car__contrilling.update_car()
                                        break
                            if car__contrilling.state!=2:
                                car__contrilling.state=2
                                car__contrilling.location=[*path[path_num+1],in_channelnum,maxdistance-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                roadmatrix1[roadchannel][roadloc] =0
                                next_roadmatrix1[in_channelnum][maxdistance-1]=carid
                                car__contrilling.update_car()
                        elif car__contrilling.state!=2:
                            car__contrilling.state=2
                            car__contrilling.location[3]=roadloc+rest_roadlen
                            roadmatrix1[roadchannel][roadloc] =0
                            roadmatrix1[roadchannel][roadloc+rest_roadlen]=carid
                        dict_1,dict_2,dict_3 =road_matrix_order(crossid_cur)
                        procar_order=[-1,-1,-1,-1]
                        carid_oder=[-1,-1,-1,-1]
                        for i in dict_3:
                            procar_order[i-1]=find_priority_car(self.road_info[dict_3[i]],car_onroad_dict)
                            carid_oder[i-1]=dict_1[i]
                        b=carid_oder[:]
                        b.sort()
                        c=[carid_oder.index(i) for i in b]
                    while(go_or_not_judge(*procar_order,c[3])):
                        print('while_4')
                        car__contrilling = procar_order[c[3]]
                        roadloc = car__contrilling.location[3]
                        roadchannel = car__contrilling.location[2]
                        path = car__contrilling.path
                        path_num = car__contrilling.path_num
                        rest_roadlen = self.road_info[path[path_num]].shape[1] - 1 - car__contrilling.location[3]
                        roadmatrix1 = self.road_info[(car__contrilling.location[0], car__contrilling.location[1])]
                        next_roadmatrix1=self.road_info[path[path_num+1]]
                        maxdistance=min(serach_roadlimitspeed(*path[path_num+1]),car__contrilling.limitspeed)-rest_roadlen
                        in_channelnum=999
                        carid = car__contrilling.carid
                        for i in range(next_roadmatrix1.shape[0]):
                            if next_roadmatrix1[i][0] !=0:
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 0:
                                    if self.control_car(car_onroad_dict[next_roadmatrix1[i][0]], car_onroad_dict)=='death_lock':
                                        return 'death_lock'
                                    if next_roadmatrix1[i][0] ==0:
                                        in_channelnum = i
                                        break
                                if car_onroad_dict[next_roadmatrix1[i][0]].state == 1:
                                    return 'death_lock'

                            else:
                                in_channelnum = i
                                break
                        if in_channelnum !=999:
                            for j in range(maxdistance):
                                if next_roadmatrix1[in_channelnum][j] !=0 :
                                    if  car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state ==0:
                                        if self.control_car(car_onroad_dict[next_roadmatrix1[in_channelnum][j]], car_onroad_dict)=='death_lock':
                                            return 'death_lock'
                                        if next_roadmatrix1[in_channelnum][j] !=0 and car__contrilling.state!=2:
                                            car__contrilling.state=2
                                            car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                            roadmatrix1[roadchannel][roadloc] =0
                                            next_roadmatrix1[in_channelnum][j]=carid
                                            car__contrilling.update_car()
                                            break
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 1:
                                        return 'death_lock'
                                    elif car_onroad_dict[next_roadmatrix1[in_channelnum][j]].state == 2 and car_contrilling.state!=2:

                                        car__contrilling.state=2
                                        car__contrilling.location=[*path[path_num+1],in_channelnum,j-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                        roadmatrix1[roadchannel][roadloc] =0
                                        next_roadmatrix1[in_channelnum][j]=carid
                                        car__contrilling.update_car()
                                        break
                            if car__contrilling.state!=2:
                                car__contrilling.state=2
                                car__contrilling.location=[*path[path_num+1],in_channelnum,maxdistance-1]  #放在前面一辆车后面，更新车的信息与车道信息
                                roadmatrix1[roadchannel][roadloc] =0
                                next_roadmatrix1[in_channelnum][maxdistance-1]=carid
                                car__contrilling.update_car()
                        elif car__contrilling.state!=2:
                            car__contrilling.state=2
                            car__contrilling.location[3]=roadloc+rest_roadlen
                            roadmatrix1[roadchannel][roadloc] =0
                            roadmatrix1[roadchannel][roadloc+rest_roadlen]=carid
                        dict_1,dict_2,dict_3 =road_matrix_order(crossid_cur)
                        procar_order=[-1,-1,-1,-1]
                        carid_oder=[-1,-1,-1,-1]
                        for i in dict_3:
                            procar_order[i-1]=find_priority_car(self.road_info[dict_3[i]],car_onroad_dict)
                            carid_oder[i-1]=dict_1[i]
                        b=carid_oder[:]
                        b.sort()
                        c=[carid_oder.index(i) for i in b]
                

def search_path(caridd):
    return path_[path_.index(caridd)+1]

def test(car_onroad_dict_m,hwmap_m):    #一秒钟调度函数，调度路上所有车一秒，测试bug用的
    car_onroad = list(car_onroad_dict_m.values())

    for i in range(len(car_onroad)):
        car_onroad[i].state = 0

    for i in range(len(car_onroad)):
        if car_onroad[i].state == 2:
            continue
        if hwmap_m.control_car(car_onroad[i], car_onroad_dict_m) == 'death_lock':
            return -1

def time(t,car_onroad_dict_m,hwmap_m):

    while(1):
        car_onroad=list(car_onroad_dict_m.values())

        for i in range(len(car_onroad)):
            car_onroad[i].state=0

        for i in range(len(car_onroad)):
            if car_onroad[i].state==2:         #状态为2，说明在之前的调度中完成调度，跳出，进行下次调度
                continue
            if hwmap_m.control_car(car_onroad[i],car_onroad_dict_m) == 'death_lock':
                return -1
            # print(i)
        t=t+1
        print('t',t)
        if len(car_onroad_dict_m) == 0:
            return t


#main主函数
hwmap=map_info(crosstxt,roadtxt)                            #首先，创建1个地图对象，由于节点和连接信息固定，地图信息固定
allcardict={i:car(search_car(i)) for i in cartxt[:,0]}      #创建所有车的dict，考虑车太多时创建较慢，可以考虑生成器的形式
global_car_onroad_dict={}                                   #创建所有在路上车的dict，便于操作
                                                            #以上的dict格式均为 车牌号:车对象
#使用方法：使用time函数，输入分别为初始时间t、global_car_onroad_dict和hwmap，输出为调度路上所有车所用的时间
#如果死锁。函数返回值为-1，可供后面算法调用api
#以下部分为算法部分，不是判别器部分，
##################################################################################
# T=10
# num= 10
# while (1):
#     for i in range(10100,10110):
#         if allcardict[i].planTime<=T and allcardict[i].yty==0:
#             hwtempmap =copy.deepcopy(hwmap)
#             car_onroad_dicttemp=copy.deepcopy(global_car_onroad_dict)
#             if add_newcar(allcardict[i],search_path(i),hwtempmap,car_onroad_dicttemp) == 'join_fail':
#                 continue
#             # bbb = time(T, car_onroad_dicttemp, hwtempmap)
#             #
#             # if bbb == -1:
#             #     continue
#             # if bbb>king and
#             #
#             #     bbb>allcardict[i].
#             #     turn=turn+1
#             #     continue
#             # king=bbb
#             add_newcar(allcardict[i],search_path(i),hwmap,global_car_onroad_dict)
#             allcardict[i].yty = 1
#             # car_onroad = list(global_car_onroad_dict.values())
#             # for i in range(len(car_onroad)):
#             #     car_onroad[i].state = 0
#             # for i in range(len(car_onroad)):
#             #     if car_onroad[i].state == 2:  # 状态为2，说明在之前的调度中完成调度，跳出，进行下次调度
#             #         continue
#             #     hwmap.control_car(car_onroad[i], global_car_onroad_dict)
#             num=num-1
#             print(i)
#     T=T+1
#     if(num==0):
#         #print(T)
#         break
