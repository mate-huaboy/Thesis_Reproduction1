#include <string>
#include<io.h>
#include <vector>
#include <iostream>

using namespace std;


 int points_num = 49;
int points_row = 7;
int points_col = 7;
double dis_center = 7;
void getFiles(string path, std::vector<string>& files, vector<string>& ownname)
{
    /*files�洢�ļ���·��������(eg.   C:\Users\WUQP\Desktop\test_devided\data1.txt)
     ownnameֻ�洢�ļ�������(eg.     data1.txt)*/

     //�ļ����
    long long   hFile = 0;
    //�ļ���Ϣ
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*.bmp").c_str(), &fileinfo)) != -1)
    {
        do
        {
            //�����Ŀ¼,����֮
            //�������,�����б�
            if ((fileinfo.attrib & _A_SUBDIR))
            {  
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    getFiles( p.assign(path).append("\\").append(fileinfo.name), files, ownname ); 
            }
            else
            {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
                ownname.push_back(fileinfo.name);
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
}