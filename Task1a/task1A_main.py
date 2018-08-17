# Team ID : 2361
# Author List : K Pranath Reddy
# Filename : task1A_main.py
# Theme : Harvestor Bot
# Functions : writecsv, main
# Global variables : 

#classes and subclasses to import
import cv2
import numpy as np
import os

#subroutine to write rerults to a csv
def writecsv(color,shape):
    #open csv file in append mode
    filep = open('results1A_2361.csv','a')
    # create string data to write per image
    datastr = "," + color + "-" + shape
    #write to csv
    filep.write(datastr)
    filep.close()

def main(path):

    lst = []
    lst1 = []
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 175,255,1)
    (_,cnts,_) = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in cnts:
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)

        if len(approx) == 3:
            shape_name = "Triangle"
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if image[cy,cx][0] == 255:
                cv2.putText(image, 'Blue-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["blue-triangle"])
                writecsv('blue','triangle')
            if image[cy,cx][1] == 255:
                cv2.putText(image, 'Green-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["green-triangle"])
                writecsv('green','triangle')
            if image[cy,cx][2] == 255:
                cv2.putText(image, 'Red-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["red-triangle"])
                writecsv('red','triangle')
        elif len(approx) == 4:
            x,y,w,h = cv2.boundingRect(cnt)
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if abs(w-h) <= 3:
                shape_name = "Square"
                if image[cy,cx][0] == 255:
                    cv2.putText(image, 'Blue-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["blue-square"])
                    writecsv('blue','square')
                if image[cy,cx][1] == 255:
                    cv2.putText(image, 'Green-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["green-square"])
                    writecsv('green','square')
                if image[cy,cx][2] == 255:
                    cv2.putText(image, 'Red-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["red-square"])
                    writecsv('red','square')
            else:
                shape_name = "Rectangle"
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if image[cy,cx][0] == 255:
                    cv2.putText(image, 'Blue-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["blue-rectangle"])
                    writecsv('blue','rectangle')
                if image[cy,cx][1] == 255:
                    cv2.putText(image, 'Green-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["green-rectangle"])
                    writecsv('green','rectangle')
                if image[cy,cx][2] == 255:
                    cv2.putText(image, 'Red-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                    lst.append(["red-rectangle"])
                    writecsv('red','rectangle')
        elif len(approx) == 5:
            shape_name = "Pentagon"
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if image[cy,cx][0] == 255:
                cv2.putText(image, 'Blue-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["blue-pentagon"])
                writecsv('blue','pentagon')
            if image[cy,cx][1] == 255:
                cv2.putText(image, 'Green-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["green-pentagon"])
                writecsv('green','pentagon')
            if image[cy,cx][2] == 255:
                cv2.putText(image, 'Red-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["red-pentagon"])
                writecsv('red','pentagon')
        elif len(approx) >= 10:
            shape_name = "Circle"
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if image[cy,cx][0] == 255:
                cv2.putText(image, 'Blue-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["blue-circle"])
                writecsv('blue','circle')
            if image[cy,cx][1] == 255:
                cv2.putText(image, 'Green-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["green-circle"])
                writecsv('green','circle')
            if image[cy,cx][2] == 255:
                cv2.putText(image, 'Red-'+shape_name, (cx-50,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                lst.append(["red-circle"])
                writecsv('red','circle')
        

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
    global results1A_2361
    mypath = './/'
    #getting all files in the directory
    onlyfiles = [os.path.join(mypath, f) for f in os.listdir(mypath) if f.endswith(".png")]
    #iterate over each file in the directory
    for fp in onlyfiles:
        #Open the csv to write in append mode
        filep = open('results1A_2361.csv','a')
        #this csv will later be used to save processed data, thus write the file name of the image 
        filep.write(fp)
        #close the file so that it can be reopened again later
        filep.close()
        #process the image
        data = main(fp)
        print data
        #open the csv
        filep = open('results1A_2361.csv','a')
        #make a newline entry so that the next image data is written on a newline
        filep.write('\n')
        #close the file
        filep.close()
