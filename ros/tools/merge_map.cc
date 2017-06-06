#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <vector>
#include<iostream>
#include<string>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <iomanip>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "Mapping2d.h"

using namespace std;

class MergeMap
{
public:
  // Enable serialization
	friend class boost::serialization::access;
	MergeMap()
	{
	  //mpMap = new ORB_SLAM2::Map();
	}
	void LoadMap(const string &filename,int i);
	void SaveMap(const string &filename);
	ORB_SLAM2::Map* mpMap;
	vector<ORB_SLAM2::Map*> mvpMap;
};


	
void MergeMap::LoadMap(const string& filename, int i)
{
    {
        std::ifstream is(filename);
        std::cout<<endl<<"Map : "<< filename<<" Loading ..."<<std::endl;
	
        boost::archive::binary_iarchive ia(is, boost::archive::no_header);
        //ia >> mpKeyFrameDatabase;
        ia >> mpMap;
       mvpMap.push_back(mpMap);
    }

    cout << filename <<" : Map Loaded!" << endl;


}

void MergeMap::SaveMap(const string &filename)
{
    cout<<endl<<"saving merge map ..."<<endl;
    std::ofstream os(filename);
    {
        ::boost::archive::binary_oarchive oa(os, ::boost::archive::no_header);
        //oa << mpKeyFrameDatabase;
        oa << mvpMap;
    }
    cout << "Map saved to " << filename << endl;

}
// 获取文件夹中文件
vector<string> getFiles(string cate_dir);

int main(int argc, char **argv)
{
  MergeMap tool;
  string MapPath = "map/";
  vector<string> MapFiles = getFiles(MapPath);
  for(unsigned i = 0; i<MapFiles.size();i++)
  {
    
    tool.LoadMap(MapPath + MapFiles[i],i);
  }
  std::cout<<"mvpMap.size : " << tool.mvpMap.size()<<std::endl;
  tool.SaveMap("Slam_Final_Map.bin");
  return 0;
}
// 获取文件夹中文件
vector<string> getFiles(string cate_dir)
{
    vector<string> files;//存放文件名

    DIR *dir;
    struct dirent *ptr;

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8)   ///file
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10)   ///link file
            continue;
        else if (ptr->d_type == 4)   ///dir
        {
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end());
    return files;
}