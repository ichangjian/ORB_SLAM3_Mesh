import os
path=""
files=os.listdir(path+"/rgb")
files.sort()
with open(path+"data.txt","w") as F:
    for file in files:
        F.write(file[:-4]+" rgb/"+file+" "+file[:-4]+" depth/"+file+"\n")