#include <cv.h>
#include <iostream>

using namespace std;
using namespace cv;
int main(int argc,char**argv){
    Mat m=Mat::ones(3,3,CV_32F);
    cout<<m<<endl;
    Mat a;
    m.convertTo(a,CV_16UC1,1e3);
    cout<<a<<endl;
    return 0;
}
