#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <ctime>

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define SERVERPORT "4950"    // the port users will be connecting to

using namespace std;
using namespace cv;
using namespace zbar;

struct Box
{
	string id;
	//CvPoint old_point[4];
	CvPoint old_point_sorted[4];
	CvPoint old_point_center;
	//CvPoint new_point[4];
	CvPoint new_point_sorted[4];
	CvPoint new_point_center;
	int length;
	int width;
	int height;
	int weight;
}BOX1, BOX2, BOX3;

struct Car
{
	string id;
	//CvPoint old_point[4];
	CvPoint old_point_sorted[4];
	CvPoint old_point_center;
	//CvPoint new_point[4];
	CvPoint new_point_sorted[4];
	CvPoint new_point_center;
	int length;
	int width;
	int height; 
	int power;
	int speed_foward;
	int speed_turn;
	double angle;
	double desiredAngle;
	int distance;
	const char *address;
}CAR1, CAR2;

int thresh = 500;
IplImage* img = 0;
IplImage* img0 = 0;
CvMemStorage* storage = 0;
CvPoint pt[4], pt_sorted[4];
const char* wndname = "Square Detection";

const int w = 640, h = 480;			
static int MAP[w][h];
static int closed_map[w][h];		
static int open_map[w][h] = {0};	
static int dir_map[w][h] = {0};	
const int dir = 8;					
static int dx[dir] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static int dy[dir] = { 0, 1, 1, 1, 0, -1, -1, -1 };


// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{   
    double dx1 = pt1->x - pt0->x;   
    double dy1 = pt1->y - pt0->y;   
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2));
}
 
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4(IplImage* img, CvMemStorage* storage)
{

    CvSeq* contours;
    int i, c, l, N = 11;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    IplImage* tgray;

    CvSeq* result;
    double s, t;

    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height )); 

    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );    

    // find squares in every color plane of the image
    for( c = 0; c < 3; c++ )
    {    
        // extract the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 ); 
        
    	// try several threshold levels   
     	for( l = 0; l < N; l++ )    
    	{        
        	// hack: use Canny instead of zero threshold level.      
       		// Canny helps to catch squares with gradient shading      
      		if( l == 0 )      
      		{
			     
            	// apply Canny. Take the upper threshold from slider 
            	// and set the lower to 0 (which forces edges merging)
            	cvCanny( tgray, gray, 0, thresh, 5 );  
                
            	// dilate canny output to remove potential   
            	// holes between edge segments 
            	cvDilate( gray, gray, 0, 1 );        
            	//cvShowImage("gray", gray); 
        	}    

        else  
        {      

    		// apply threshold if l!=0:
            // tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0 
            cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY ); 
        } 

        // find contours and store them all as a list
        cvFindContours( gray, storage, &contours, sizeof(CvContour),  
        CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );  

        // test each contour
        	while( contours )
            {
            	// approximate contour with accuracy proportional
                // to the contour perimeter
                result = cvApproxPoly( contours, sizeof(CvContour), storage,
                CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                
                if( result->total == 4 &&
                    fabs(cvContourArea(result,CV_WHOLE_SEQ)) > 1000 &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;
                    for( i = 0; i < 5; i++ )
                    {

                    	// find minimum angle between joint 
						// edges (maximum of cosine)
                        if( i >= 2 )

                        {
                            t = fabs(angle(
                            (CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
                            s = s > t ? s : t;
                        }
                    }
            
                // if cosines of all angles are small 
                // (all angles are ~90 degree) then write quandrange
                // vertices to resultant sequence
                	if( s < 0.3 )
                        for( i = 0; i < 4; i++ ) 
                           cvSeqPush( squares, 
                               (CvPoint*)cvGetSeqElem( result, i ));
                }
                // take the next contour 
            	contours = contours->h_next;
            }
        }
    }
    // release all the temporary images
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );
    
    return squares;
}

// the function draws all the squares in the image
void drawSquares( IplImage* img, CvSeq* squares )
{   
    CvSeqReader reader;
    IplImage* cpy = cvCloneImage( img );
    int i;

    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );
    
    // read 4 sequence elements at a time (all vertices of a square)
    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint* rect = pt;
        int count = 4;
 
        // read 4 vertices
        memcpy( pt, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );
        
        memcpy( pt + 1, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );

        memcpy( pt + 2, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );

        memcpy( pt + 3, reader.ptr, squares->elem_size );
        CV_NEXT_SEQ_ELEM( squares->elem_size, reader );

        // draw the square as a closed polyline
        cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,0,255), 2, CV_AA, 0 );
    }

	// show the resultant image
	cvShowImage( wndname, cpy );
	cvReleaseImage( &cpy );
}

void PointSort()
{
	int Sum = 0, Sub = 0;

	Sum = pt[0].x + pt[0].y;
		pt_sorted[0] = pt[0];
		for(int j=1; j<4; j++)
		{
			if(pt[j].x + pt[j].y < Sum)
			{
				Sum = pt[j].x + pt[j].y;
				pt_sorted[0] = pt[j];
			}
		}

		Sub = pt[0].y - pt[0].x;
		pt_sorted[1] = pt[0]; 
		for(int j=1; j<4; j++)
		{
			if(pt[j].y - pt[j].x < Sub)
			{
				Sub = pt[j].y - pt[j].x;
				pt_sorted[1] = pt[j];
			}
		}


		Sum = pt[0].x + pt[0].y;
		pt_sorted[2] = pt[0];
		for(int j=1; j<4; j++)
		{
			if(pt[j].x + pt[j].y > Sum)
			{
				Sum = pt[j].x + pt[j].y;
				pt_sorted[2] = pt[j];
			}
		}
	
		Sub = pt[0].y - pt[0].x;
		pt_sorted[3] = pt[0]; 
		for(int j=1; j<4; j++)
		{
			if(pt[j].y - pt[j].x > Sub)
			{
				Sub = pt[j].y - pt[j].x;
				pt_sorted[3] = pt[j];
			}
		}

		/*cout << pt[0].x << " " << pt[0].y << " ";
		cout << pt[1].x << " " << pt[1].y << " ";
		cout << pt[2].x << " " << pt[2].y << " "; 
		cout << pt[3].x << " " << pt[3].y << endl;
		cout << pt_sorted[0].x << " " << pt_sorted[0].y << " ";
		cout << pt_sorted[1].x << " " << pt_sorted[1].y << " ";
		cout << pt_sorted[2].x << " " << pt_sorted[2].y << " ";
		cout << pt_sorted[3].x << " " << pt_sorted[3].y << endl << endl;	*/
		
}

IplImage* WarpPerspective(IplImage* input)
{
	IplImage* output;
	output = cvCreateImage(cvGetSize(input), IPL_DEPTH_8U, 3);

	CvMat* mmat = cvCreateMat(3,3,CV_32FC1);
    
	CvPoint2D32f *c1 = new CvPoint2D32f[4];
	CvPoint2D32f *c2 = new CvPoint2D32f[4];
	
	c1[0].x = pt_sorted[0].x;	c1[0].y = pt_sorted[0].y;
	c1[1].x = pt_sorted[1].x;	c1[1].y = pt_sorted[1].y;
	c1[2].x = pt_sorted[2].x;	c1[2].y = pt_sorted[2].y;
	c1[3].x = pt_sorted[3].x;	c1[3].y = pt_sorted[3].y;
	
	c2[0].x = 10;	c2[0].y = 10;
	c2[1].x = 100;	c2[1].y = 10;
	c2[2].x = 100;	c2[2].y = 100;
	c2[3].x = 10;	c2[3].y = 100;
    
	mmat = cvGetPerspectiveTransform(c1, c2, mmat);
    cvWarpPerspective(input, output, mmat);

	/*cout << pt[0].x << " " << pt[0].y << " ";
	cout << pt[1].x << " " << pt[1].y << " ";
	cout << pt[2].x << " " << pt[2].y << " "; 
	cout << pt[3].x << " " << pt[3].y << endl;
	cout << pt_sorted[0].x << " " << pt_sorted[0].y << " ";
	cout << pt_sorted[1].x << " " << pt_sorted[1].y << " ";
	cout << pt_sorted[2].x << " " << pt_sorted[2].y << " ";
	cout << pt_sorted[3].x << " " << pt_sorted[3].y << endl << endl;*/

	//cvShowImage("Original", input);
	//cvShowImage("Warp", output);

	return output;
}

IplImage* imageROI(IplImage* warp_img)
{
	IplImage* warp_img_ROI;

	CvRect rect_ROI = cvRect(10, 10 , 90, 90);

	warp_img_ROI = cvCloneImage(warp_img);
	cvSetImageROI(warp_img_ROI, rect_ROI);

	//cvShowImage("ROI", warp_img_ROI);

	return warp_img_ROI;
}

string QRcode_Decode(IplImage* warp_img_ROI)
{
	ImageScanner scanner;    
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);   
    
	Mat img_ROI = cvarrToMat(warp_img_ROI);  
	//cvResetImageROI(warp_img_ROI);
	Mat imgout;
	cvtColor(img_ROI,img_ROI,CV_RGB2GRAY);
	//threshold(img_ROI, img_ROI, 127, 255, CV_THRESH_BINARY);

	imgout = img_ROI;
    
	string qrcode_data;
	int width = img_ROI.cols;    
	int height = img_ROI.rows;    
	uchar *raw = (uchar *)img_ROI.data;       
	Image image(width, height, "Y800", raw, width * height);      
	int n = scanner.scan(image);      
	for(Image::SymbolIterator symbol = image.symbol_begin();symbol != image.symbol_end();++symbol)  
	{    
		vector<Point> vp;    
		cout<<"Decoded："<<endl<<symbol->get_type_name()<<endl<<endl;  
		cout<<"Symbol："<<endl<<symbol->get_data()<<endl<<endl;           
		int n = symbol->get_location_size();    
		for(int i=0;i<n;i++)  
		{    
			vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));   
		}    
		RotatedRect r = minAreaRect(vp);    
        
		qrcode_data=symbol->get_data();     
		
		putText(cvarrToMat(img),qrcode_data,pt_sorted[0],FONT_HERSHEY_COMPLEX,1,Scalar(0,0,255),1,8,false);
	}    
    

	zbar_image_free_data(image);
	img_ROI.release();
	imgout.release();
	return qrcode_data;
}

double line_inverseTri(int beginX, int beginY, int endX, int endY)
{
	double x1 = beginX,  y1 = beginY;
	double x2 = endX,    y2 = endY;
	double x  = x2 - x1, y  = -(y2 - y1); 
	double theta = 0;
	theta = atan(y/x) * 180 / 3.1415926;

	cout << theta << endl;

	if(x > 0 && y > 0)
		theta = theta;

	if(x < 0 && y > 0)
		theta = theta + 180;

	if(x < 0 && y < 0)
		theta = theta + 180;

	if(x > 0 && y < 0)
		theta = theta + 360;

	cout << theta << endl;

	return theta;
}

double circle_inverseTri(int oldCenterX, int oldCenterY, int newCenterX, int newCenterY, int beginX, int beginY, int endX, int endY)
{
	if(abs(oldCenterX - newCenterX) >= 2 || abs(oldCenterY - newCenterY) >= 2)
		return 0;

	double x0 = newCenterX, y0 = newCenterY;
	double x1 = beginX , y1 = beginY;
	double x2 = endX   , y2 = endY;
	double x  = x2 -x1 , y = -(y2 - y1);

	if(abs(x) <= 2 && abs(y) <= 2)
		return 0;

	double denominator = ((x1 - x0) * (x2 - x0)) + ((y1 - y0) * (y2 - y0));
	double numerator   = sqrt(((x1 - x0) * (x1 - x0)) + ((y1 - y0) * (y1 - y0))) *
						 sqrt(((x2 - x0) * (x2 - x0)) + ((y2 - y0) * (y2 - y0)));
	double param = denominator / numerator;
	cout << param << endl;

	if(param == 1)
		param = 0.9995;

	double theta = 0;
	if(numerator != 0)
		theta = acos(param) * 180 / 3.1415926;
	
	if(x > 0 && y > 0)
		theta = 0 - theta;
	if(x < 0 && y < 0)
		theta = theta;
	if(theta < 0.1 && theta > -0.1)
		theta = 0;
	if(theta > 90  || theta < -90)
		theta = 0;

	cout << theta << endl;

	if(theta > 80)
		theta = 90 - theta;
	if(theta < -80)
		theta = -90 -theta;

	cout << "theta 90" << theta << endl;

	return theta;
}

string moveControl(int beginX, int beginY, int endX, int endY, int speed_foward)
{
	int time;
	string time_str;
	string message_str;
	
	int x1 = beginX, y1 = beginY;
	int x2 = endX,   y2 = endY;
	int distancce = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
	time = distance * 1000 / speed_foward;
	
	time_str = to_string(time);
	
}

string turnControl(double desiredAngle, double angle, int speed_turn)
{
	int time;
	string time_str;
	string message_str;
	
	if(abs(desiredAngle - angle) > 5 && (desiredAngle - angle) > 0 && 
	   (desiredAngle - angle) < 180) //turn left
	{
		time = (desiredAngle - angle) * 1000 / speed_turn;
		time_str = to_string(time);
		message_str.append("L");
		message_str.append(time_str);			
		return message_str;
	}
	
	if(abs(desiredAngle - angle) > 5 && (desiredAngle - angle) < -180 && 
	   (desiredAngle - angle) > -360) //turn left
	{
		time = abs(desiredAngle - angle) * 1000 / speed_turn;
		time_str = to_string(time);
		message_str.append("L");
		message_str.append(time_str);			
		return message_str;
	}
	
	if(abs(desiredAngle - angle) > 5 && (desiredAngle - angle) > 180 && 
	   (desiredAngle - angle) < 360) //turn Right
	{
		time = (desiredAngle - angle) * 1000 / speed_turn;
		time_str = to_string(time);
		message_str.append("R");
		message_str.append(time_str);			
		return message_str;
	}
	
	if(abs(desiredAngle - angle) > 5 && (desiredAngle - angle) < 0 && 
	   (desiredAngle - angle) > -180) //turn Right
	{
		time = abs(desiredAngle - angle) * 1000 / speed_turn;
		time_str = to_string(time);
		message_str.append("R");
		message_str.append(time_str);			
		return message_str;
	}				
}


void MatchItemInfo(string qrcode_data)
{
	CvFont font = cvFont(1, 1);
	string subdata, subdata_idType, subdata_id;
	double theta = 0;

	subdata_idType = qrcode_data.substr(0, 1);
	if(subdata_idType == "B")
	{
		subdata_id = qrcode_data.substr(0, 2);
		
		if(BOX1.id == subdata_id)
		{	
			BOX1.old_point_sorted[0] = BOX1.new_point_sorted[0];
			BOX1.old_point_sorted[1] = BOX1.new_point_sorted[1];
			BOX1.old_point_sorted[2] = BOX1.new_point_sorted[2];
			BOX1.old_point_sorted[3] = BOX1.new_point_sorted[3];

			BOX1.old_point_center.x = (BOX1.old_point_sorted[0].x + BOX1.old_point_sorted[1].x +
									   BOX1.old_point_sorted[2].x + BOX1.old_point_sorted[3].x ) / 4;

			BOX1.old_point_center.y = (BOX1.old_point_sorted[0].y + BOX1.old_point_sorted[1].y +
									   BOX1.old_point_sorted[2].y + BOX1.old_point_sorted[3].y ) / 4;


			BOX1.new_point_sorted[0] = pt_sorted[0];
			BOX1.new_point_sorted[1] = pt_sorted[1];
			BOX1.new_point_sorted[2] = pt_sorted[2];
			BOX1.new_point_sorted[3] = pt_sorted[3];

			BOX1.new_point_center.x = (BOX1.new_point_sorted[0].x + BOX1.new_point_sorted[1].x +
									   BOX1.new_point_sorted[2].x + BOX1.new_point_sorted[3].x ) / 4;

			BOX1.new_point_center.y = (BOX1.new_point_sorted[0].y + BOX1.new_point_sorted[1].y +
									   BOX1.new_point_sorted[2].y + BOX1.new_point_sorted[3].y ) / 4;
		}

		if(BOX2.id == subdata_id)
		{
			BOX2.old_point_sorted[0] = BOX2.new_point_sorted[0];
			BOX2.old_point_sorted[1] = BOX2.new_point_sorted[1];
			BOX2.old_point_sorted[2] = BOX2.new_point_sorted[2];
			BOX2.old_point_sorted[3] = BOX2.new_point_sorted[3];

			BOX2.old_point_center.x = (BOX2.old_point_sorted[0].x + BOX2.old_point_sorted[1].x +
									   BOX2.old_point_sorted[2].x + BOX2.old_point_sorted[3].x ) / 4;

			BOX2.old_point_center.y = (BOX2.old_point_sorted[0].y + BOX2.old_point_sorted[1].y +
									   BOX2.old_point_sorted[2].y + BOX2.old_point_sorted[3].y ) / 4;


			BOX2.new_point_sorted[0] = pt_sorted[0];
			BOX2.new_point_sorted[1] = pt_sorted[1];
			BOX2.new_point_sorted[2] = pt_sorted[2];
			BOX2.new_point_sorted[3] = pt_sorted[3];

			BOX2.new_point_center.x = (BOX2.new_point_sorted[0].x + BOX2.new_point_sorted[1].x +
									   BOX2.new_point_sorted[2].x + BOX2.new_point_sorted[3].x ) / 4;

			BOX2.new_point_center.y = (BOX2.new_point_sorted[0].y + BOX2.new_point_sorted[1].y +
									   BOX2.new_point_sorted[2].y + BOX2.new_point_sorted[3].y ) / 4;


		}

		if(BOX3.id == subdata_id)
		{
			BOX3.old_point_sorted[0] = BOX3.new_point_sorted[0];
			BOX3.old_point_sorted[1] = BOX3.new_point_sorted[1];
			BOX3.old_point_sorted[2] = BOX3.new_point_sorted[2];
			BOX3.old_point_sorted[3] = BOX3.new_point_sorted[3];

			BOX3.old_point_center.x = (BOX3.old_point_sorted[0].x + BOX3.old_point_sorted[1].x +
									   BOX3.old_point_sorted[2].x + BOX3.old_point_sorted[3].x ) / 4;

			BOX3.old_point_center.y = (BOX3.old_point_sorted[0].y + BOX3.old_point_sorted[1].y +
									   BOX3.old_point_sorted[2].y + BOX3.old_point_sorted[3].y ) / 4;


			BOX3.new_point_sorted[0] = pt_sorted[0];
			BOX3.new_point_sorted[1] = pt_sorted[1];
			BOX3.new_point_sorted[2] = pt_sorted[2];
			BOX3.new_point_sorted[3] = pt_sorted[3];

			BOX3.new_point_center.x = (BOX3.new_point_sorted[0].x + BOX3.new_point_sorted[1].x +
									   BOX3.new_point_sorted[2].x + BOX3.new_point_sorted[3].x ) / 4;

			BOX3.new_point_center.y = (BOX3.new_point_sorted[0].y + BOX3.new_point_sorted[1].y +
									   BOX3.new_point_sorted[2].y + BOX3.new_point_sorted[3].y ) / 4;


		}
	}


	if(subdata_idType == "C")
	{
		subdata_id = qrcode_data.substr(0, 2);
		
		if(CAR1.id == subdata_id)
		{
			CAR1.old_point_sorted[0] = CAR1.new_point_sorted[0];
			CAR1.old_point_sorted[1] = CAR1.new_point_sorted[1];
			CAR1.old_point_sorted[2] = CAR1.new_point_sorted[2];
			CAR1.old_point_sorted[3] = CAR1.new_point_sorted[3];

			CAR1.old_point_center.x = (CAR1.old_point_sorted[0].x + CAR1.old_point_sorted[1].x +
									   CAR1.old_point_sorted[2].x + CAR1.old_point_sorted[3].x ) / 4;

			CAR1.old_point_center.y = (CAR1.old_point_sorted[0].y + CAR1.old_point_sorted[1].y +
									   CAR1.old_point_sorted[2].y + CAR1.old_point_sorted[3].y ) / 4;

			/*for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++)
				{
					if(abs(pt_sorted[j].x - CAR1.old_point_sorted[i].x) < 10 &&  abs(pt_sorted[j].x - CAR1.old_point_sorted[i].x) < 10)
						CAR1.new_point_sorted[i] = pt_sorted[j];
				}
			}*/

			//if(CAR1.new_point_center.x == 0 || CAR1.new_point_center.y == 0)
			//{
				CAR1.new_point_sorted[0] = pt_sorted[0];
				CAR1.new_point_sorted[1] = pt_sorted[1];
				CAR1.new_point_sorted[2] = pt_sorted[2];
				CAR1.new_point_sorted[3] = pt_sorted[3];
			//}

			CAR1.new_point_center.x = (CAR1.new_point_sorted[0].x + CAR1.new_point_sorted[1].x +
									   CAR1.new_point_sorted[2].x + CAR1.new_point_sorted[3].x ) / 4;

			CAR1.new_point_center.y = (CAR1.new_point_sorted[0].y + CAR1.new_point_sorted[1].y +
									   CAR1.new_point_sorted[2].y + CAR1.new_point_sorted[3].y ) / 4;

			cvPutText(img, "pt0", cvPoint(CAR1.new_point_sorted[0].x, CAR1.new_point_sorted[0].y), &font, CV_RGB(255, 0, 0));
			cvPutText(img, "pt1", cvPoint(CAR1.new_point_sorted[1].x, CAR1.new_point_sorted[1].y), &font, CV_RGB(255, 0, 0));
			cvPutText(img, "pt2", cvPoint(CAR1.new_point_sorted[2].x, CAR1.new_point_sorted[2].y), &font, CV_RGB(255, 0, 0));
			cvPutText(img, "pt3", cvPoint(CAR1.new_point_sorted[3].x, CAR1.new_point_sorted[3].y), &font, CV_RGB(255, 0, 0));

			theta = circle_inverseTri(CAR1.old_point_center.x, CAR1.old_point_center.y,
									  CAR1.new_point_center.x, CAR1.new_point_center.y,
									  CAR1.old_point_sorted[0].x, CAR1.old_point_sorted[0].y,
									  CAR1.new_point_sorted[0].x, CAR1.new_point_sorted[0].y);

			CAR1.angle = CAR1.angle + theta;
			
			if(CAR1.angle > 360)
				CAR1.angle = CAR1.angle - 360;
			if(CAR1.angle < 0)
				CAR1.angle = CAR1.angle + 360;
		}

		if(CAR2.id == subdata_id)
		{
			CAR2.old_point_sorted[0] = CAR2.new_point_sorted[0];
			CAR2.old_point_sorted[1] = CAR2.new_point_sorted[1];
			CAR2.old_point_sorted[2] = CAR2.new_point_sorted[2];
			CAR2.old_point_sorted[3] = CAR2.new_point_sorted[3];

			CAR2.old_point_center.x = (CAR2.old_point_sorted[0].x + CAR2.old_point_sorted[1].x +
									   CAR2.old_point_sorted[2].x + CAR2.old_point_sorted[3].x ) / 4;

			CAR2.old_point_center.y = (CAR2.old_point_sorted[0].y + CAR2.old_point_sorted[1].y +
									   CAR2.old_point_sorted[2].y + CAR2.old_point_sorted[3].y ) / 4;


			CAR2.new_point_sorted[0] = pt_sorted[0];
			CAR2.new_point_sorted[1] = pt_sorted[1];
			CAR2.new_point_sorted[2] = pt_sorted[2];
			CAR2.new_point_sorted[3] = pt_sorted[3];

			CAR2.new_point_center.x = (CAR2.new_point_sorted[0].x + CAR2.new_point_sorted[1].x +
									   CAR2.new_point_sorted[2].x + CAR2.new_point_sorted[3].x ) / 4;

			CAR2.new_point_center.y = (CAR2.new_point_sorted[0].y + CAR2.new_point_sorted[1].y +
									   CAR2.new_point_sorted[2].y + CAR2.new_point_sorted[3].y ) / 4;
		}
	}
	
	/*cout << "BOX1 ID: " << BOX1.id << endl;
	cout << "BOX1 Length:" << BOX1.length <<endl;
	cout << "BOX1 Width :" << BOX1.width  <<endl;
	cout << "BOX1 Height:" << BOX1.height <<endl;
	cout << "BOX1 Weight:" << BOX1.weight <<endl;
	cout << BOX1.old_point_sorted[0].x << " " << BOX1.old_point_sorted[0].y << " ";
	cout << BOX1.old_point_sorted[1].x << " " << BOX1.old_point_sorted[1].y << " ";
	cout << BOX1.old_point_sorted[2].x << " " << BOX1.old_point_sorted[2].y << " ";
	cout << BOX1.old_point_sorted[3].x << " " << BOX1.old_point_sorted[3].y << " " << endl;
	cout << BOX1.old_point_center.x << BOX1.old_point_center.y << endl;
	cout << BOX1.new_point_sorted[0].x << " " << BOX1.new_point_sorted[0].y << " ";
	cout << BOX1.new_point_sorted[1].x << " " << BOX1.new_point_sorted[1].y << " ";
	cout << BOX1.new_point_sorted[2].x << " " << BOX1.new_point_sorted[2].y << " ";
	cout << BOX1.new_point_sorted[3].x << " " << BOX1.new_point_sorted[3].y << " " << endl;
	cout << BOX1.new_point_center.x <<  " " << BOX1.new_point_center.y << endl;*/

	/*cout << "BOX2 ID: " << BOX2.id << endl;
	cout << "BOX2 Length:" << BOX2.length <<endl;
	cout << "BOX2 Width :" << BOX2.width  <<endl;
	cout << "BOX2 Height:" << BOX2.height <<endl;
	cout << "BOX2 Weight:" << BOX2.weight <<endl;
	cout << BOX2.old_point_sorted[0].x << " " << BOX2.old_point_sorted[0].y << " ";
	cout << BOX2.old_point_sorted[1].x << " " << BOX2.old_point_sorted[1].y << " ";
	cout << BOX2.old_point_sorted[2].x << " " << BOX2.old_point_sorted[2].y << " ";
	cout << BOX2.old_point_sorted[3].x << " " << BOX2.old_point_sorted[3].y << " " << endl;
	cout << BOX2.old_point_center.x << BOX2.old_point_center.y << endl;
	cout << BOX2.new_point_sorted[0].x << " " << BOX2.new_point_sorted[0].y << " ";
	cout << BOX2.new_point_sorted[1].x << " " << BOX2.new_point_sorted[1].y << " ";
	cout << BOX2.new_point_sorted[2].x << " " << BOX2.new_point_sorted[2].y << " ";
	cout << BOX2.new_point_sorted[3].x << " " << BOX2.new_point_sorted[3].y << " " << endl;
	cout << BOX2.new_point_center.x <<  " " << BOX2.new_point_center.y << endl;*/


	/*cout << "BOX3 ID: " << BOX3.id << endl;
	cout << "BOX3 Length:" << BOX3.length <<endl;
	cout << "BOX3 Width :" << BOX3.width  <<endl;
	cout << "BOX3 Height:" << BOX3.height <<endl;
	cout << "BOX3 Weight:" << BOX3.weight <<endl;
	cout << BOX3.old_point_sorted[0].x << " " << BOX3.old_point_sorted[0].y << " ";
	cout << BOX3.old_point_sorted[1].x << " " << BOX3.old_point_sorted[1].y << " ";
	cout << BOX3.old_point_sorted[2].x << " " << BOX3.old_point_sorted[2].y << " ";
	cout << BOX3.old_point_sorted[3].x << " " << BOX3.old_point_sorted[3].y << " " << endl;
	cout << BOX3.old_point_center.x << BOX3.old_point_center.y << endl;
	cout << BOX3.new_point_sorted[0].x << " " << BOX3.new_point_sorted[0].y << " ";
	cout << BOX3.new_point_sorted[1].x << " " << BOX3.new_point_sorted[1].y << " ";
	cout << BOX3.new_point_sorted[2].x << " " << BOX3.new_point_sorted[2].y << " ";
	cout << BOX3.new_point_sorted[3].x << " " << BOX3.new_point_sorted[3].y << " " << endl;
	cout << BOX3.new_point_center.x <<  " " << BOX3.new_point_center.y << endl;*/


	cout << "CAR1 ID: " << CAR1.id << endl;
	cout << "CAR1 Length:" << CAR1.length <<endl;
	cout << "CAR1 Width :" << CAR1.width  <<endl;
	cout << "CAR1 Height:" << CAR1.height <<endl;
	cout << "CAR1 Power :" << CAR1.power <<endl;
	cout << CAR1.old_point_sorted[0].x << " " << CAR1.old_point_sorted[0].y << " ";
	cout << CAR1.old_point_sorted[1].x << " " << CAR1.old_point_sorted[1].y << " ";
	cout << CAR1.old_point_sorted[2].x << " " << CAR1.old_point_sorted[2].y << " ";
	cout << CAR1.old_point_sorted[3].x << " " << CAR1.old_point_sorted[3].y << " " << endl;
	cout << CAR1.old_point_center.x << CAR1.old_point_center.y << endl;
	cout << CAR1.new_point_sorted[0].x << " " << CAR1.new_point_sorted[0].y << " ";
	cout << CAR1.new_point_sorted[1].x << " " << CAR1.new_point_sorted[1].y << " ";
	cout << CAR1.new_point_sorted[2].x << " " << CAR1.new_point_sorted[2].y << " ";
	cout << CAR1.new_point_sorted[3].x << " " << CAR1.new_point_sorted[3].y << " " << endl;
	cout << CAR1.new_point_center.x <<  " " << CAR1.new_point_center.y << endl;
	cout << CAR1.angle << endl;
	//cout << CAR2.angle << endl;

	/*cout << "CAR2 ID: " << CAR2.id << endl;
	cout << "CAR2 Length:" << CAR2.length <<endl;
	cout << "CAR2 Width :" << CAR2.width  <<endl;
	cout << "CAR2 Height:" << CAR2.height <<endl;
	cout << "CAR2 Power :" << CAR2.power <<endl;
	cout << CAR2.old_point_sorted[0].x << " " << CAR2.old_point_sorted[0].y << " ";
	cout << CAR2.old_point_sorted[1].x << " " << CAR2.old_point_sorted[1].y << " ";
	cout << CAR2.old_point_sorted[2].x << " " << CAR2.old_point_sorted[2].y << " ";
	cout << CAR2.old_point_sorted[3].x << " " << CAR2.old_point_sorted[3].y << " " << endl;
	cout << CAR2.old_point_center.x << CAR2.old_point_center.y << endl;
	cout << CAR2.new_point_sorted[0].x << " " << CAR2.new_point_sorted[0].y << " ";
	cout << CAR2.new_point_sorted[1].x << " " << CAR2.new_point_sorted[1].y << " ";
	cout << CAR2.new_point_sorted[2].x << " " << CAR2.new_point_sorted[2].y << " ";
	cout << CAR2.new_point_sorted[3].x << " " << CAR2.new_point_sorted[3].y << " " << endl;
	cout << CAR2.new_point_center.x <<  " " << CAR2.new_point_center.y << endl;*/
}


//A* Path Finding//
class node
{
	int xp, yp;	// current posistion
	int level;		// move distance
	int priority;  

public:
	node(int xP, int yP, int d, int p)
	{
		xp = xP; yp = yP; level = d; priority = p;
	}

	int getxPos() const { return xp; }
	int getyPos() const { return yp; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int & xDest, const int & yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; 
	}

	
	void nextLevel(const int & i) 
	{
		level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	
	const int & estimate(const int & dx, const int & dy) const
	{
		static int d = static_cast<int>(sqrt((dx - xp)*(dx - xp) + (dy - yp)*(dy - yp)));
		return(d);
	}
};

bool operator<(const node & a, const node & b)
{
	return a.getPriority() > b.getPriority();
}

string pathFind(const int & x1, const int & y1, const int & x2, const int & y2)
{
	static priority_queue<node> pq[2];
	static int pqi;
	static node* w0;
	static node* h0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;
	
	w0 = new node(x1, y1, 0, 0);
	w0->updatePriority(x2, y2);
	pq[pqi].push(*w0);
	open_map[x][y] = w0->getPriority(); 
	
	while (!pq[pqi].empty()){
		
		w0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
		x = w0->getxPos(); y = w0->getyPos();
		pq[pqi].pop();				
		open_map[x][y] = 0;
		closed_map[x][y] = 1;		
		
		if (x == x2 && y == y2){
			string path = "";
			while (!(x == x1 && y == y1)){
				j = dir_map[x][y];
				c = '0' + (j + dir / 2) % dir;
				path = c + path;
				x += dx[j];
				y += dy[j];
			}
			delete w0;
			while (!pq[pqi].empty()) pq[pqi].pop();		
			return path;
		}
		
		for (i = 0; i<dir; i++){
			xdx = x + dx[i]; ydy = y + dy[i];
			
			if (!(xdx<0 || xdx>w - 1 || ydy<0 || ydy>h - 1 || MAP[xdx][ydy] == 1 || closed_map[xdx][ydy] == 1)){
				h0 = new node(xdx, ydy, w0->getLevel(), w0->getPriority());
				h0->nextLevel(i);
				h0->updatePriority(x2, y2);
				
				if (open_map[xdx][ydy] == 0){
					open_map[xdx][ydy] = h0->getPriority();
					pq[pqi].push(*h0);
					dir_map[xdx][ydy] = (i + dir / 2) % dir;	
				}
				else if (open_map[xdx][ydy]>h0->getPriority()){
					open_map[xdx][ydy] = h0->getPriority();		
					dir_map[xdx][ydy] = (i + dir / 2) % dir;	

					while (!(pq[pqi].top().getxPos() == xdx && pq[pqi].top().getyPos() == ydy)){
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop();									
					
					if (pq[pqi].size()>pq[1 - pqi].size()) pqi = 1 - pqi;
					while (!pq[pqi].empty()){
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*h0); 
				}
				else delete h0;
			}
		}
		delete w0;
	}
}

void loadMap(IplImage* img)
{
	for (int y = 0; y < 480; y++)
	{
		for (int x = 0; x < 640; x++)
		{
			if(CV_IMAGE_ELEM(img, uchar, y, 3*x) == 255)
			{
				//cout << x << " " << y << endl;
				MAP[x][y] = 1;
			}
			else{
				MAP[x][y] = 0;
			}
		}
	}
}

int drawPath(IplImage* img, string route, int StartX, int StartY)
{
	/*int StartX = Start.x, StartY = Start.y;
	int GoalX = Goal.x, GoalY = Goal.y;*/
	
	if (route.length() == 0) return -1;
	int j, j1; char c, c1;
	int x = StartX, x1;
	int y = StartY, y1;
	
	for (int i = 0; i<route.length()-1; i++)
	{
		c = route.at(i);
		j = atoi(&c);
		x = x + dx[j];
		y = y + dy[j];

		c1 = route.at(i+1);
		j1 = atoi(&c);
		x1 = x + dx[j];
		y1 = y + dy[j];

		cvLine(img, Point(x,y), Point(x1,y1), Scalar(0,0,200), 1);
	}
}

int socket_test(const char *address, string message_str)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
	//char *address;
	const char *message;

	//address = "192.168.1.103";
	message = message_str.c_str();

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

	
		if ((rv = getaddrinfo(address, SERVERPORT, &hints, &servinfo)) != 0) {
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
			return 1;
		}

		// loop through all the results and make a socket
		for(p = servinfo; p != NULL; p = p->ai_next) {
			if ((sockfd = socket(p->ai_family, p->ai_socktype,
					p->ai_protocol)) == -1) {
				perror("talker: socket");
				continue;
			}

			break;
		}

		if (p == NULL) {
			fprintf(stderr, "talker: failed to create socket\n");
			return 2;
		}

		if ((numbytes = sendto(sockfd, message, strlen(message), 0,
				 p->ai_addr, p->ai_addrlen)) == -1) {
			perror("talker: sendto");
			exit(1);
		}
		printf("talker: sent %c, %d bytes to %s\n", message, numbytes, address);
		sleep(5);
	

    freeaddrinfo(servinfo);

    close(sockfd);

    return 0;
}




void on_trackbar( int a )
{
	if( img )
    	drawSquares( img, findSquares4( img, storage ) );
}

int main(int argc, char** argv)
{
	string qrcode_data;

	BOX1.id = "B1"; 
	BOX1.length = 10;	BOX1.width = 15;	BOX1.height = 10;	BOX1.weight = 20;

	BOX2.id = "B2"; 
	BOX2.length = 20;	BOX2.width = 30;	BOX2.height = 30;	BOX2.weight = 40;

	BOX3.id = "B3";
	BOX3.length = 30;	BOX3.width = 50;	BOX3.height = 30;	BOX3.weight = 90;

	CAR1.id = "C1";
	CAR1.length = 50;	CAR1.width = 50;	CAR1.height = 50;	CAR1.power  = 50;
	CAR1.speed_foward = 50;		CAR1.speed_turn   = 50;
	CAR1.angle = 0;		CAR1.desiredAngle = 0;		CAR1.distance = 0;
	CAR1.address = "192.168.1.103";
	
	CAR2.id = "C2";
	CAR2.length = 50;	CAR2.width = 50;	CAR2.height = 50;	CAR2.power  = 50;
	CAR2.speed_foward = 50;		CAR2.speed_turn   = 50;
	CAR2.angle = 0;		CAR2.desiredAngle = 0;		CAR2.distance = 0;
	CAR2.address = "192.168.1.114";

	int car1Time = 2000;
	int car2Time = 2500;
	string car1Message_str;
	string car2Message_str;
	string direction_F = "F";
	string direction_L = "L";
	string direction_R = "R";
	string direction_B = "B";
	string car1Time_str = to_string(car1Time);
	string car2Time_str = to_string(car2Time);
	car1Message_str.append("H");
	car1Message_str.append(car1Time_str);
	car2Message_str.append(direction_B);
	car2Message_str.append(car2Time_str);
	
	
	const char *car1Address = CAR1.address;
	//const char *car1Message = car1Message_str.c_str();
	const char *car2Address = CAR2.address;
	//const char *car2Message = car2Message_str.c_str();

	//int StartX =  10, StartY =  10;
	//int GoalX  = 615, GoalY  = 465;
	//string route;

	// create memory storage that will contain all the dynamic data
	storage = cvCreateMemStorage(0);

	CvCapture* cap = cvCreateCameraCapture(0);
	img = cvQueryFrame(cap);
	//img = cvCloneImage( img0 );
	IplImage* warp_img;
	IplImage* warp_img_ROI;

	// create window and a trackbar (slider) with parent "image" and set callback
	// (the slider regulates upper threshold, passed to Canny edge detector)
	cvNamedWindow( wndname, 1 );
	cvCreateTrackbar( "canny thresh", wndname, &thresh, 1000, on_trackbar );

	// force the image processing
	on_trackbar(0);

	//cvRectangle(img, Point(100,100), Point(150,150), Scalar(255,0,0));
	//cvRectangle(img, Point(100,180), Point(150,210), Scalar(255,0,0));
	//cvRectangle(img, Point(100,200), Point(220,250), Scalar(255,0,0));
	//cvRectangle(img, Point(300,100), Point(350,350), Scalar(255,0,0));
	//cvRectangle(img, Point(500,400), Point(550,450), Scalar(255,0,0));

	//loadMap(img);
	//route = pathFind(StartX, StartY, GoalX, GoalY);	

	while(true)
	{
		socket_test(car1Address, car1Message_str);
		socket_test(car2Address, car2Message_str);
		storage = cvCreateMemStorage(0);

		img = cvQueryFrame(cap);
		IplImage* warp_img;
		IplImage* warp_img_ROI;

		if( img )
		{
			drawSquares( img, findSquares4( img, storage ) );
			PointSort();
			warp_img = WarpPerspective(img);
			warp_img_ROI = imageROI(warp_img);
			qrcode_data = QRcode_Decode(warp_img_ROI);
			
			if(qrcode_data != "")
				MatchItemInfo(qrcode_data);
			
			if(BOX1.new_point_center.x != 0 && BOX1.new_point_center.y != 0 &&
			   BOX2.new_point_center.x != 0 && BOX2.new_point_center.y != 0 &&
			   CAR1.new_point_center.x != 0 && CAR1.new_point_center.y != 0 &&
			   CAR2.new_point_center.x != 0 && CAR2.new_point_center.y != 0)
			{
				CAR1.angle = circle_inverseTri(CAR1.old_point_center.x, CAR1.old_point_center.y,
											   CAR1.new_point_center.x, CAR1.new_point_center.y,
											   CAR1.old_point_sorted[0].x, CAR1.old_point_sorted[0].y,
											   CAR1.new_point_sorted[0].x, CAR1.new_point_sorted[0].y);
				CAR2.angle = circle_inverseTri(CAR2.old_point_center.x, CAR2.old_point_center.y,
											   CAR2.new_point_center.x, CAR2.new_point_center.y,
											   CAR2.old_point_sorted[0].x, CAR2.old_point_sorted[0].y,
											   CAR2.new_point_sorted[0].x, CAR2.new_point_sorted[0].y);
				if((CAR1.desiredAngle - CAR1.angle) > 5 )
				{
					CAR1.desiredAngle = line_inverseTri(CAR1.new_point_center.x, 
														CAR1.new_point_center.y, 
														BOX2.new_point_center.x, 
														BOX2.new_point_center.y);
				}
				if((CAR2.desiredAngle - CAR2.angle) > 5)
				{
					CAR2.desiredAngle = line_inverseTri(CAR2.new_point_center.x, 
														CAR2.new_point_center.y, 
														BOX2.new_point_center.x, 
														BOX2.new_point_center.y);
				}
				
				car1Message_str = turnControl(CAR1.desiredAngle, CAR1.angle, CAR1.speed_turn);
				car2Message_str = turnControl(CAR2.desiredAngle, CAR2.angle, CAR2.speed_turn);
				
				socket_test(car1Address, car1Message_str);
				socket_test(car2Address, car2Message_str);
				
				car1Message_str = moveControl();
				
				
			}
			
		}
		char c = cvWaitKey(1);
		cvReleaseImage(&warp_img);
		cvReleaseImage(&warp_img_ROI);
		cvReleaseImage(&img0);

		cvReleaseMemStorage( &storage ); 		
		if(c == 27)
			break;
	}
	cvReleaseImage(&img);
	return 0;
}
