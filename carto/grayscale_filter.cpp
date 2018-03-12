#include <stdlib.h>
#include <iostream>
#include <string>
#include "CartoImageProc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "PerlinNoise.hpp"
#include "json.hpp"

using namespace Carto;
using namespace cv;
using json = nlohmann::json;

void refresh_window(int pos, void *userData);
void init_window(int window_number);
void refresh_preview();
void refresh_path();
void decrement_preview(Mat *edges, Mat *preview, uchar amount);
void waitLoop();
void save(void *userdata);
void load(void *userdata);
void init_control_panel();

int start = 0, end = 150, total_layers=2;
int starts[]= {1,1,1}; // Leave at 1 to pass getTrackbarPos test
int ends[] = {255,255,255};
int perlin_scales[] = {25,25,25};
int curr_window=0;
int perlin_scale=25;

std::string *config_file;

Mat imgmat1, imgmat2, imgmat3, imgmat4 ,canny, preview, preview_tsp;
Mat window_img[3], edges[3];

CartoImageProc *img;
CartoImageProc *procs[3];

void refresh(int pos, void *userData);
int main( int argc, char** argv ){
    config_file = new std::string(argv[2]);
    
    std::cout << cv::getBuildInformation() << std::endl;
    //img=new CartoImageProc("/Users/matt/xcode/Cartogrifer/carto/carto/img.jpg");
    img=new CartoImageProc(argv[1]);
    img->toGrayscale();
    
    procs[0] = new CartoImageProc(argv[1], 0);
    procs[1] = new CartoImageProc(argv[1], 1);
    procs[2] = new CartoImageProc(argv[1], 2);
    
    procs[0]->toGrayscale();
    procs[1]->toGrayscale();
    procs[2]->toGrayscale();
    
    for(int i=0;i<total_layers;i++) {
        init_window((int)i);
    }
    
    init_control_panel();

    waitLoop();
}

void waitLoop() {
    while(1){
        char key=waitKey(0);
        
        switch(key) {
            case 'r': {
                refresh_preview();
                break;
            }
            case 'p': {
                refresh_preview();
                refresh_path();
                break;
            }default: { exit(0); }
        }
    }
}
void init_window(int win_number) {
    Mat *i;
    CartoImageProc *p;
    int window_number=win_number;
    curr_window=win_number;
    
    string window_name="Step ";
    window_name.append(std::to_string(window_number));

    p=procs[window_number];
    
    i = &window_img[window_number];
    
    *i=p->mat.clone();
    
    p->filterGrayscale(i,starts[window_number],ends[window_number]);
    
    //if(p->id == 0) {
        p->filterPerlin(i,(double)perlin_scales[window_number]/100);
    //}
    
    edges[window_number]=i->clone();
    Canny(edges[window_number],edges[window_number],1,3,3);
    bitwise_not(edges[window_number],edges[window_number]);
    
    p->show(*i,window_name);
    
    //refresh_preview();
    
    if(!(getTrackbarPos("Start",window_name) > 0)) {
        createTrackbar("Start",window_name,&starts[window_number],255,refresh_window,(void *)&p->id);
        createTrackbar("End",window_name,&ends[window_number],255,refresh_window,&p->id);
        createTrackbar("Perlin",window_name,&perlin_scales[window_number],100,refresh_window,(void *)&p->id);
    }
}

void refresh_window(int pos, void *userdata) {
    //int window = *((int *)&userdata);
    int *window = reinterpret_cast<int *>(userdata);
    
    int num=*window;
    init_window(num);
    
    return;
}

void save(void *userdata) {
    json j;
    std::string file = procs[0]->image_name;
    int start,end,perlin;
    
    std::ifstream in(config_file->c_str());
    
    if(in.good()) {
        in >> j;
    }
    
    in.close();
    
    for(int i=0; i<total_layers; i++) {
        string window_name="Step ";
        window_name.append(std::to_string(i));
        start=getTrackbarPos("Start", window_name);
        end=getTrackbarPos("End", window_name);
        perlin=getTrackbarPos("Perlin", window_name);
        j[file]["windows"][i]["name"]=window_name;
        j[file]["windows"][i]["trackbars"]= {
            { "Start", start },
            { "End", end },
            { "Perlin", perlin }
        };
    }
    
    std::ofstream out(config_file->c_str());
    out << std::setw(4) << j << std::endl;
    out.close();
}

void load(void *userdata) {
    json j;
    std::string file=procs[0]->image_name;
    
    std::ifstream in(config_file->c_str());
    
    if(in.good()) {
        in >> j;
        in.close();
    }else{
        in.close();
        return;
    }
    
    for(int i=0; i < j[file]["windows"].size(); i++) {
        std::string window_name=j[file]["windows"][i]["name"].get<std::string>();
        //for(int k=0; k < j[file]["windows"][i]["trackbars"].size(); k++){
            //json o = j[file]["windows"][i]["trackbars"][k];
            
            for(json::iterator it = j[file]["windows"][i]["trackbars"].begin(); it != j[file]["windows"][i]["trackbars"].end(); ++it){
                setTrackbarPos(it.key(), window_name, it.value());
           }
        //}
    }
}

void init_control_panel() {
    createButton("Save", (ButtonCallback)save);
    createButton("Load", (ButtonCallback)load);
}

void refresh_preview() {
    preview=edges[0].clone();
    preview=Scalar::all(255);

    decrement_preview(&edges[0],&preview,50);
    decrement_preview(&edges[1],&preview,100);
    
    //img->buildPath(&preview);
    img->show(preview,"Preview");
}

void refresh_path() {
    img->buildPath(&preview);
    img->show(preview,"Path");
    
    /* TSP 
     preview_tsp=edges[1].clone();
     edges[1].copyTo(preview_tsp);
     img->buildTSPath(&preview_tsp);
     //img->show(preview,"Path");
     img->show(preview_tsp,"TSP Path");
     
     */
}

void decrement_preview(Mat *edges, Mat *preview, uchar amount){
    uchar *eval,*pval;
    
    for(int x=0; x < edges->cols; x++){
        for(int y=0; y < edges->rows; y++) {
            eval=&edges->at<uchar>(Point(x,y));
            pval=&preview->at<uchar>(Point(x,y));

            if(*eval < 255 && *pval > amount) {
                *pval-=amount;
            }
        }
    }
}

void refresh(int pos, void *userData) {
    imgmat1 = img->mat.clone();
    img->filterGrayscale(&imgmat1, start,end);
    img->filterPerlin(&imgmat1,(double)perlin_scale/100);

    img->show(imgmat1,"0 to 150");
    
    canny=imgmat1.clone();
    Canny(canny,canny,1,3,3);
    bitwise_not(canny,canny);
    
    img->show(canny,"Scratch");
}
