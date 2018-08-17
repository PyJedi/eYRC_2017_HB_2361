# Team ID : 2361
# Author List : K Pranath Reddy
# Filename : task1B_main.py
# Theme : Harvestor Bot
# Functions : writecsv, main
# Global variables : 

#classes and subclasses to import
import cv2
import numpy as np
import os

#subroutine to write rerults to a csv
def writecsv(color,shape,size,count):
    #open csv file in append mode
    filep = open('results1B_2361.csv','a')
    # create string data to write per image
    datastr = "," + color + "-" + shape + "-" + size + "-" + count
    #write to csv
    filep.write(datastr)
    filep.close()

def main(path):

    lst = []
    lst1 = []
    j=0
    k=1
    count=[0]*75
    image = cv2.imread(path)
    kernel = np.ones((5,5),np.float32)/25
    dst = cv2.filter2D(image,-1,kernel)
    gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
    
    thresh=cv2.Canny(image,100,200)
   
    ret, thresh = cv2.threshold(thresh, 127,255,cv2.THRESH_BINARY)
    
    (_, cnts, hierarchy) = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    L=(len(cnts))/2
    

    for cnt in cnts:
        k=k+1
        if k%2 != 0:
            approx = cv2.approxPolyDP(cnt, 0.019*cv2.arcLength(cnt,True),True)
            if len(approx) == 3:
                shape_name = "Triangle" 
                q=cnt.shape              
                roi = cnt[int(q[0]/2):q[0],int(q[1]/2):q[1]]
                cx1= roi[0][0][0]
                cy1= roi[0][0][1]
                M =cv2.moments(roi)
                cx2=int(M['m10']/M['m00'])
                cy2=int(M['m01']/M['m00'])
                cx=(cx1+cx2)/2
                cy=(cy1+cy2)/2
                area = cv2.contourArea(cnt)
                if image[cy,cx][0] == 255:
                    if area >= 5663.5:
                        cv2.putText(image, 'Large-Blue-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-blue-triangle"])
                        count[0] = count[0]+1
                    elif area<=1585.0:
                        cv2.putText(image, 'Small-Blue-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-blue-triangle"])
                        count[1] = count[1]+1
                    else:
                        cv2.putText(image, 'medium-Blue-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-blue-triangle"])
                        count[2] = count[2]+1
                elif image[cy,cx][1] == 255:
                    if area >= 5663.5:
                        cv2.putText(image, 'Large-Green-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-green-triangle"])
                        count[3] = count[3]+1
                    elif area<=1585.0:
                        cv2.putText(image, 'Small-Green-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-green-triangle"])
                        count[4] = count[4]+1
                    else:
                        cv2.putText(image, 'Medium-Green-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-green-triangle"])
                        count[5] = count[5]+1
                elif image[cy,cx][2] == 255 and image[cy,cx][0] == 0 and image[cy,cx][1] == 0 :
                    if area >= 5663.5:
                        cv2.putText(image, 'Large-Red-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-red-triangle"])
                        count[6] = count[6]+1
                    elif area<=1585.0:
                        cv2.putText(image, 'Small-Red-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-red-triangle"])
                        count[7] = count[7]+1
                    else:
                        cv2.putText(image, 'Medium-Red-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-red-triangle"])
                        count[8] = count[8]+1
                elif image[cy,cx][2] == 255 and image[cy,cx][1] == 255 :
                    if area >= 5663.5:
                        cv2.putText(image, 'Large-yellow-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-yellow-triangle"])
                        count[9] = count[9]+1
                    elif area<=1585.0:
                        cv2.putText(image, 'Small-yellow-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-yellow-triangle"])
                        count[10] = count[10]+1
                    else:
                        cv2.putText(image, 'Medium-yellow-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["Medium-yellow-triangle"])
                        count[11] = count[11]+1                       
                elif  50<= image[cy,cx][1] <= 200 and 0<= image[cy,cx][0] <=50 and image[cy,cx][2] == 255:
                    if area >= 5663.5:
                        cv2.putText(image, 'Large-orange-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-orange-triangle"])
                        count[12] = count[12]+1
                    elif area<=1585.0:
                        cv2.putText(image, 'Small-orange-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-orange-triangle"])
                        count[13] = count[13]+1
                    else:
                        cv2.putText(image, 'Medium-orange-'+shape_name, (cx-5,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-orange-triangle"])
                        count[14] = count[14]+1
            elif len(approx) == 4:
                x,y,w,h = cv2.boundingRect(cnt)
                q=cnt.shape
                roi = cnt[int(q[0]/2):q[0],:q[1]]
                cx1=roi[0][0][0]
                cy1=roi[0][0][1]
                M = cv2.moments(roi)
                cx2=int(M['m10']/M['m00'])
                cy2=int(M['m01']/M['m00'])
                cx=(cx1+cx2)/2
                cy=(cy1+cy2)/2
                area = cv2.contourArea(cnt)
                if abs(w-h) <= 3:
                    shape_name = "Square"
                    if image[cy,cx][0] == 255:
                        if area >=10813.0:
                            cv2.putText(image, 'Large-Blue-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-blue-square"])
                            count[15] = count[15]+1
                        elif area<=2913.0:
                           cv2.putText(image, 'Small-Blue-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                           lst.append(["small-blue-square"])
                           count[16] = count[16]+1
                        else:
                            cv2.putText(image, 'Medium-Blue-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-blue-square"])
                            count[17] = count[17]+1
                    elif image[cy,cx][1] == 255:
                        if area >=10813.0:
                            cv2.putText(image, 'Large-Green-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-green-square"])
                            count[18] = count[18]+1
                        elif area<=2913.0:
                            cv2.putText(image, 'Small-Green-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-green-square"])
                            count[19] = count[19]+1
                        else:
                            cv2.putText(image, 'Medium-Green-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-green-square"])
                            count[20] = count[20]+1
                    elif image[cy,cx][2] == 255 and image[cy,cx][0] == 0 and image[cy,cx][1] == 0:
                        if area >=10813.0:
                            cv2.putText(image, 'Large-Red-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-red-square"])
                            count[21] = count[21]+1
                        elif area<=2913.0:
                            cv2.putText(image, 'Small-Red-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-red-square"])
                            count[22] = count[22]+1
                        else:
                            cv2.putText(image, 'Medium-Red-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-red-square"])
                            count[23] = count[23]+1
                    elif image[cy,cx][2] == 255 and image[cy,cx][1] == 255 :
                        if area >=10813.0:
                            cv2.putText(image, 'Large-yellow-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-yellow-Square"])
                            count[24] = count[24]+1
                        elif area<=2913.0:
                            cv2.putText(image, 'Small-yellow-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-yellow-Square"])
                            count[25] = count[25]+1
                        else:
                            cv2.putText(image, 'Medium-yellow-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-yellow-Square"])
                            count[26] = count[26]+1
                    elif  50<= image[cy,cx][1] <= 200 and 0<= image[cy,cx][0] <=50 and image[cy,cx][2] == 255 :
                        if area >=10813.0:
                            cv2.putText(image, 'Large-orange-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-orange-Square"])
                            count[27] = count[27]+1
                        elif area<=2913.0:
                            cv2.putText(image, 'Small-orange-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-orange-Square"])
                            count[28] = count[28]+1
                        else:
                            cv2.putText(image, 'Medium-orange-'+shape_name, (cx-15,cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-orange-Square"])
                            count[29] = count[29]+1
                            
                else:
                    shape_name = "Rectangle"
                    q=cnt.shape
                    roi = cnt[int(q[0]/1.5):q[0],:q[1]]
                    cx1=cnt[0][0][0]
                    cy1=cnt[0][0][1]
                    M = cv2.moments(roi)
                    cx2=int(M['m10']/M['m00'])
                    cy2=int(M['m01']/M['m00'])
                    cx=(cx1+cx2)/2
                    cy=(cy1+cy2)/2
                    area = cv2.contourArea(cnt)
                    if image[cy,cx][0]==255:
                        if area >= 21213.0:
                            cv2.putText(image, 'Large-Blue-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-blue-rectangle"])
                            count[30] = count[30]+1
                        elif area<=3993.0:
                            cv2.putText(image, 'Small-Blue-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-blue-rectangle"])
                            count[31] = count[31]+1
                        else:
                            cv2.putText(image, 'Medium-Blue-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-blue-rectangle"])
                            count[32] = count[32]+1
                    elif image[cy,cx][1]==255 and image[cy,cx][0]==0 and image[cy,cx][2]==0:
                        if area >= 21213.0:
                            cv2.putText(image, 'Large-Green-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-green-rectangle"])
                            count[33] = count[33]+1
                        elif area<=3993.0:
                            cv2.putText(image, 'Small-Green-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-green-rectangle"])
                            count[34] = count[34]+1
                        else:
                            cv2.putText(image, 'Medium-Green-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-green-rectangle"])
                            count[35]= count[35]+1
                    elif image[cy,cx][2] == 255 and image[cy,cx][0] == 0 and image[cy,cx][1] == 0:
                        if area >= 21213.0:
                            cv2.putText(image, 'Large-Red-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-red-rectangle"])
                            count[36] = count[36]+1
                        elif area<=3993.0:
                            cv2.putText(image, 'Small-Red-'+shape_name,(cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-red-rectangle"])
                            count[37] = count[37]+1
                        else:
                            cv2.putText(image, 'Medium-Red-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-red-rectangle"])
                            count[38] = count[38]+1
                    elif image[cy,cx][1] == 255 and image[cy,cx][2]==255:
                        if area >= 21213.0:
                            cv2.putText(image, 'Large-Yellow-'+shape_name,(cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-yellow-Rectangle"])
                            count[39] = count[39]+1
                        elif area<=3993.0:
                            cv2.putText(image, 'Small-Yellow-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-yellow-Rectangle"])
                            count[40] = count[40]+1
                        else:
                            cv2.putText(image, 'Medium-Yellow-'+shape_name,(cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-yellow-Rectangle"])
                            count[41] = count[41]+1
                    elif 50<= image[cy,cx][1] <= 200 and 0<= image[cy,cx][0] <=50 and image[cy,cx][2] <= 255:
                        if area >= 21213.0:
                            cv2.putText(image, 'Large-orange-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["large-orange-Rectangle"])
                            count[42] = count[42]+1
                        elif area<=3993.0:
                            cv2.putText(image, 'Small-orange-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["small-orange-Rectangle"])
                            count[43] = count[43]+1
                        else:
                            cv2.putText(image, 'Medium-orange-'+shape_name, (cx,cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                            lst.append(["medium-orange-Rectangle"])
                            count[44] = count[44]+1
            elif len(approx) >= 7:
                shape_name = "Circle"
                q=cnt.shape
                roi = cnt[int(q[0]/2):q[0],int(q[1]/2):int(q[1])]
                cx1=roi[0][0][0]
                cy1=roi[0][0][1]
                M = cv2.moments(roi)
                cx2=int(M['m10']/M['m00'])
                cy2=int(M['m01']/M['m00'])
                cx=(cx1+cx2)/2
                cy=(cy1+cy2)/2
                area = cv2.contourArea(cnt)
                if image[cy,cx][0] == 255:
                    if area>=8497.0:
                        cv2.putText(image, 'Large-Blue-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-blue-circle"])
                        count[45] = count[45]+1
                    elif area<=2286.5:
                        cv2.putText(image, 'Small-Blue-'+shape_name,(cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-blue-circle"])
                        count[46] = count[46]+1
                    else:
                        cv2.putText(image, 'Medium-Blue-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-blue-circle"])
                        count[47] = count[47]+1
                elif image[cy,cx][1] == 255:
                    if area>=8497.0:
                        cv2.putText(image, 'Large-Green-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-green-circle"])
                        count[48] = count[48]+1
                    elif area<=2286.5:
                        cv2.putText(image, 'Small-Green-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-green-circle"])
                        count[49] = count[49]+1
                    else:
                        cv2.putText(image, 'Medium-Green-'+shape_name,(cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-green-circle"])
                        count[50] = count[50]+1
                elif image[cy,cx][2] == 255 and image[cy,cx][0] == 0 and image[cy,cx][1] == 0:
                    if area>=8497.0:
                        cv2.putText(image, 'Large-Red-'+shape_name,(cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-red-circle"])
                        count[51] = count[51]+1
                    elif area<=2286.5:
                        cv2.putText(image, 'Small-Red-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-red-circle"])
                        count[52] = count[52]+1
                    else:
                        cv2.putText(image, 'Medium-Red-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-red-circle"])
                        count[53] = count[53]+1
                elif image[cy,cx][2] == 255 and image[cy,cx][1] == 255 :
                    if area>=8497.0:
                        cv2.putText(image, 'Large-yellow-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-yellow-Circle"])
                        count[54] = count[54]+1
                    elif area<=2286.5:
                        cv2.putText(image, 'Small-yellow-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-yellow-Circle"])
                        count[55] = count[55]+1
                    else:
                        cv2.putText(image, 'Medium-yellow-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-yellow-Circle"])
                        count[56] = count[56]+1
                elif 50<= image[cy,cx][1] <= 200 and 0<= image[cy,cx][0] <=50 and image[cy,cx][2] == 255 :
                    if area>=8497.0:
                        cv2.putText(image, 'Large-orange-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["large-orange-Circle"])
                        count[57] = count[57]+1
                    elif area<=2286.5:
                        cv2.putText(image, 'Small-orange-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["small-orange-Circle"])
                        count[58] = count[58]+1
                    else:
                        cv2.putText(image, 'Medium-orange-'+shape_name, (cx-30,cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0),1)
                        lst.append(["medium-orange-Circle"])
                        count[59] = count[59]+1

    color = [1]*5
    color[0] = 'Blue'
    color[1] = 'Green'
    color[2] = 'Red'
    color[3] = 'Yellow'
    color[4] = 'Orange'
    shape = [1]*4
    shape[0] = 'Triangle'
    shape[1] = 'Square'
    shape[2] = 'Rectangle'
    shape[3] = 'Circle'
    size = [1]*3
    size[0] = 'Large'
    size[1] = 'Small'
    size[2] = 'Medium'
    l=0
    for i in range(0,4):
        for j in range(0,5):
            for k in range(0,3):
                if count[l] != 0:
                    writecsv(color[j],shape[i],size[k],str(count[l]))
                l=l+1

    cv2.imwrite(path.replace(".//test","output"),image)    
    lst1.append(path)
    for i in range(0,len(lst)):
            x=lst[i]
            lst1.append(x)
    print lst1

    cv2.drawContours(image, cnts, -1, (0,0,0), 2)
    cv2.imshow('Identifying Colour and Shapes', image)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
            
#main where the path is set for the directory containing the test images
if __name__ == "__main__":
    global results1B_2361
    mypath = './/'
    #getting all files in the directory
    onlyfiles = [os.path.join(mypath, f) for f in os.listdir(mypath) if f.endswith(".png")]
    #iterate over each file in the directory
    for fp in onlyfiles:
        #Open the csv to write in append mode
        filep = open('results1B_2361.csv','a')
        #this csv will later be used to save processed data, thus write the file name of the image 
        filep.write(fp)
        #close the file so that it can be reopened again later
        filep.close()
        #process the image
        data = main(fp)
        print data
        #open the csv
        filep = open('results1B_2361.csv','a')
        #make a newline entry so that the next image data is written on a newline
        filep.write('\n')
        #close the file
        filep.close()

