''' daisha_PC_v2.0 ######################################################################

・RTKモジュールから、位置情報を取得する。
・取得した位置情報(スタートとゴールの座標)から、直線経路の方程式（ax+by+c=0）を算出する。
・直線経路からのずれ量を計算し、マイコンに送信する。
・左折用の経路に，始点と終点をつなぎ変える

######################################################################################### '''

import socket       # ソケット通信用モジュール
import math     # 数学関係モジュール 
import serial       # シリアル通信用モジュール
import time     # 時間関数用モジュール 
import re       #北緯と東経を分離するために使用

# ###### --左折用(カンマ区切り版)-- 出発点と目的地(スタート→中間点)　########################################

#座標（スタート，中間点1,中間点2...,ゴール）のテキストファイルを開く
fp = open('route.txt', 'r', encoding='UTF-8')

#ファイルを文字列として読み込む
route_txt=fp.read()

fp.close()

#それぞれの行のデータをリストの一要素にする．
route_data_set=route_txt.splitlines()
print(route_data_set)
pattern="(.*),(.*),(.*)"

Point_NUM=len(route_data_set)
print("経路上の点の個数：{0}個".format(Point_NUM))
print("")

#ファイルから取得した座標を表示する．
for i in range(Point_NUM):
    Point_info=re.search(pattern, route_data_set[i])
    Label=Point_info.group(1); LAT=Point_info.group(2); LNG=Point_info.group(3)
    print("{0}: 北緯{1}, 東経{2}".format(Label,LAT,LNG))

print("")

def check_fix():    # fixしているか確認
    print("\n現在fixしているか確認しています...")
    while True:
        codelist_check=get_codelist()   # $GNGGAセンテンスを取得 
        if int(codelist_check[6])==4:
            print("現在fixしています。\n")
            # time.sleep(1) # 1秒処理停止
            break

def min_deg(phi_min):#　分から度に変換
    # 緯度、経度共に dddmm.mmmm表記なので100で割って分をすべて小数点以下にする
    f,i=math.modf(phi_min/100.0)  # i 整数部（ddd度） , f 小数部（.mmmmmm)
    phi_deg=i+f*100.0/60.0 # ddd.dddd度表記になる
    return phi_deg
    
def deg_rad(phi_deg):#　度からラジアンに変換 
    phi_rad=phi_deg*math.pi/180.0
    return phi_rad

def get_course(LAT_s,LNG_s,LAT_f,LNG_f):# コースの数式係数を取得 
    # LAT_s 緯度スタート, LNG_s 経度スタート
    # LAT_f 緯度終点, LNG_f 経度終点
    R=6378100.0 #　地球半径[m]
    dy=deg_rad(LAT_f - LAT_s)*R # y方向変位（緯度）
    dx=deg_rad(LNG_f - LNG_s)*R # x方向変位（経度）
    course=[ dy , -dx , 0.0 ]   # ax+by=0(原点を通るから)　右方向のずれが正
    return course

def get_codelist(): #　$GNGGAセンテンスを取得 
    while True:
        j=0 # $の場所の定義
        data1 = str(s.recv(300)) # データ 300バイト分取得
        while True:
            data2 = str(s.recv(300))
            data1 += data2  # データを追加する
            if len(data2) < 300: # １パケット分を取得したらループ抜ける
               break 

        j=data1.find("$GNGGA")  # 取得データから$GNGGAの頭文字$の要素番号を取得
        code=data1[j:j+88]      # $GNGGAの頭文字要素数から88文字目までを読み込む
        if(code.count("$")==1)and(code.count("'")==0):
            # 欠陥がなければちょうど88文字で $マークが1個
            # なぜか'が入るときがあり，エラーが出るから
            codelist=code.split(',')    # $GNGGAセンテンスを','区切りでリスト化
            break
    return codelist # コードのリストを返す

def get_d(a,b,c,LAT,LNG,LAT_s,LNG_s): #経路からのずれの量を取得
    R=6378100.0  #地球半径[m]
    y=deg_rad(LAT - LAT_s)*R  #現在地と出発地のy方向変位
    x=deg_rad(LNG - LNG_s)*R  #現在地と出発地のx方向変位
    d = (a*x + b*y + 0.0) / math.sqrt(a*a + b*b) #経路からのずれ量
    return d #経路からのずれ量を返す

def get_limit_d(LAT,LNG,LAT_f,LNG_f): #経路からのずれの量を取得
    R=6378100.0  #地球半径[m]
    y=deg_rad(LAT - LAT_f)*R  #現在地と出発地のy方向変位
    x=deg_rad(LNG - LNG_f)*R  #現在地と出発地のx方向変位
    limit_d = math.sqrt(x*x + y*y) #目的地からの距離
    return limit_d #目的地からの距離  

def get_edge(LAT1,LNG1,LAT2,LNG2):  # ２地点の距離を取得
    R=6373100.  #地球半径[m]
    dy=deg_rad(LAT2 - LAT1)*R
    dx=deg_rad(LNG2 - LNG1)*R
    edge=math.sqrt((dx)*(dx) + (dy)*(dy))
    return edge

limit_d=20          # ずれ量許容範囲[m]
limit_LL=0.0000004      #　緯度経度許容範囲[deg]

ser = serial.Serial('COM6',115200,timeout = 0.1)   # シリアル通信開始 ここにはArduinoのシリアルポート番号を設定する．

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:    # ソケット通信開始
    s.connect(('localhost',50000))      # IPf vvvvcaアドレス、ポート番号指定
    check_fix()                 # fixしているか確認

    #input("計測を開始するときはEnterを押してください。")
    print("計測開始")
    print("出発点から動きます")
    

    time.sleep(36)
    starttime=time.time()

    flag=0
    flag2=0
    #i=0
    for i in range(Point_NUM-1):

        print("")

        Point_info_s=re.search(pattern, route_data_set[i])
        Point_info_f=re.search(pattern, route_data_set[i+1])
        Label_s=Point_info_s.group(1); LAT_s=float(Point_info_s.group(2)); LNG_s=float(Point_info_s.group(3))
        Label_f=Point_info_f.group(1); LAT_f=float(Point_info_f.group(2)); LNG_f=float(Point_info_f.group(3)) 
        print("{0}から{1}に向かって走行中".format(Label_s,Label_f))
        print("{0}: 北緯{1}, 東経{2}".format(Label_s,LAT_s,LNG_s))
        print("{0}: 北緯{1}, 東経{2}".format(Label_f,LAT_f,LNG_f))

        course=get_course(LAT_s,LNG_s,LAT_f,LNG_f)  # コースの数式係数を取得
        a=course[0];b=course[1];c=course[2]     # ax+by+c=0
        limit_d=20

        j=0
        k=0
        while True:
            #print("内側ループ:",j,sep='')
            j=j+1
            codelist=get_codelist()
            Time=round((float(codelist[1])/10000+9),4)  # 日本時間
            NS=codelist[3]          #  北緯南緯
            LAT=min_deg(float(codelist[2])) #　緯度の単位を度に変換
            EW=codelist[5]          # 東経西経
            LNG=min_deg(float(codelist[4])) # 経度の単位を度に変換
            FF=int(codelist[6])     # fixかfloatか : fixは 4, floatは 5を判断
            

            if FF==4:               # fixは 4, floatは 5を判断

                if limit_d <= 0.4:
                    flag=1
                    time.sleep(10)
                    print("ストップ")
                    break

                d=get_d(a,b,c,LAT,LNG,LAT_s,LNG_s)  # 経路からのずれの量[m]を取得
                edge=get_edge(LAT_s,LNG_s,LAT,LNG)  # 出発地と現在地の距離を取得
                limit_d=get_limit_d(LAT,LNG,LAT_f,LNG_f) # 目的地からの距離[m]を取得  

                currenttime=time.time()
                #print("limit_d={0}".format(limit_d))
                
                if(currenttime-starttime > 3.5):    #　2秒毎にシリアル通信 走行用の周期は3秒
                    k=k+1

                    if  flag2==1:
                        print("d=0による走行を終了しました．")
                        flag2=0

                    starttime=currenttime
                    
                    d_cm=d*100 #ずれ量を[m]から[cm]に変換

                    if d_cm>=127:
                        d_cm=127
                    
                    if d_cm<=-127:
                        d_cm=-127

                    #目的地に到着したときは，ずれ量がないと仮定する
                    if flag==1:
                        d_cm=-128
                        print("d={0}に初期化して走行中(3秒間)".format(d))
                        flag2=1
                        #time.sleep(3)
                        flag=0

                    d_int=int(d_cm)
                    if d_int<0:
                        d_int=256-abs(d_int)

                    bina_d=bytes([d_int])

                    print(k,LAT,LNG,limit_d,d,sep=",")

                    ser.write(bina_d) #マイコンに値を書き込むと，車が動き始める

                    time.sleep(0.1)
                    c = ser.read() #マイコンから値を読み取ってくる


                    
            else:
                print("float")  #　Fix解以外をまとめてFloat解とする
                continue
