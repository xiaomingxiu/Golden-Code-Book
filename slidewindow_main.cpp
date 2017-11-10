// sobel

// Author: Xiao Ming Xiu
// Date: 10.27   2017


											#define HAVE_OPENCV_FLANN
											#define HAVE_OPENCV_IMGPROC
											#define HAVE_OPENCV_VIDEO
											#define HAVE_OPENCV_OBJDETECT
											#define HAVE_OPENCV_CALIB3D
											#define HAVE_OPENCV_ML
											#define HAVE_OPENCV_HIGHGUI
											#define HAVE_OPENCV_CONTRIB
											#define HAVE_OPENCV_PHOTO
											#define HAVE_OPENCV_FEATURES2D


											#include<opencv2/opencv.hpp>
											#include<opencv2/core/core_c.h>
											#include<opencv2/core/core.hpp>

											# include "opencv2/core/core.hpp"
											# include "opencv2/features2d/features2d.hpp"
											# include "opencv2/highgui/highgui.hpp"
											# include "opencv2/calib3d/calib3d.hpp"
											# include "opencv2/nonfree/features2d.hpp"


											#include "opencv2/opencv_modules.hpp"

										
											#include <iostream>
											#include <fstream>
											#include<string>
											#include <iomanip>

											#include <opencv2/core/core.hpp>


											#include <opencv2/calib3d/calib3d.hpp>
											#include <opencv2/highgui//highgui.hpp>

											#include "dmtx.h"   ////////////////////////////////////////////////dmtx 

											#include <cstdio>
											#include <ctime>



											#include<stdio.h>
											#include<string.h>

											#include<stdio.h>
											#include<sys/time.h>
											#include<unistd.h>










#include<math.h>
#include<opencv2/opencv.hpp>

#include<iostream>

#include<vector>      









using namespace std;
using namespace cv;

const float PI=3.14159265;


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&     Global Var Define   &&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&						

typedef struct LineSeg_
{
	Point Seed;
	Point LeftEnd;
	Point RightEnd;
	float Average_Theta;	

//--------------
	float Length;
	float StepLen;
	
}LineSeg;



typedef struct Ten_Rits_
{
	Point2f Origin;
	Point2f StepVec1;
	Point2f StepVec2;

//---------------for RS-Decoding---------

	Point2f A;
	Point2f B;
//---------------for Geometric-----------		
}Ten_Rits;








void printLineSeg(LineSeg ThisLine)
{
	cout<<endl;
	cout<<endl;
	cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
	cout<<"^^^^^^         Line Seg                ^^^^"<<endl;
	cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
	cout<<endl;
	cout<<endl;
	
	cout<<"ThisLine.Seed="<<ThisLine.Seed<<endl;
	cout<<"ThisLine.LeftEnd="<<ThisLine.LeftEnd<<endl;
	cout<<"ThisLine.RightEnd="<<ThisLine.RightEnd<<endl;
	cout<<"ThisLine.Average_Theta="<<ThisLine.Average_Theta<<endl;

	cout<<"-----------------------------"<<endl;

	cout<<"ThisLine.Length="<<ThisLine.Length<<endl;
	cout<<"ThisLine.StepLen="<<ThisLine.StepLen<<endl;

	cout<<endl;
	cout<<endl;
	cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
	cout<<"^^^^^^   End of Line Seg               ^^^^"<<endl;
	cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
	cout<<endl;
	cout<<endl;

}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&     End of Global Var Define   &&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&





//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

						//==================================================================================
						//==================================================================================
						// Basic Math
		
						//		
						double distanceOfTwoPoints(Point2f A, Point2f B)
						{
							return sqrt( (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) );	
						}
							


						// End of Basic Math
						//==================================================================================
						//==================================================================================








						//==================================================================================
						//==================================================================================
						// care the difference of  |x(i)-x(i-1)|
						void longitudeScan(Mat& img, Mat& diff)
						{	
						
							for(int i=1;i<img.rows-1;i++)
								for(int j=1;j<img.cols-1;j++)
									{
										diff.at<float>(i,j)= (float)  ((  2* (float)img.at<uchar>(i,j+1) - 2*(float)img.at<uchar>(i,j-1)
														+ 1* (float)img.at<uchar>(i+1,j+1) - 1*(float)img.at<uchar>(i+1,j-1)
														+ 1* (float)img.at<uchar>(i-1,j+1) - 1*(float)img.at<uchar>(i-1,j-1) )/8.0);

									}		
							
						}
						//==================================================================================
						//==================================================================================






						//==================================================================================
						//==================================================================================
						// care the difference of  |x(i)-x(i-1)|
						void verticalScan(Mat& img, Mat& diff)
						{	
							for(int j=1;j<img.cols-1;j++)
								for(int i=1;i<img.rows-1;i++)
									{
										diff.at<float>(i,j)= (float) ((  2* (float)img.at<uchar>(i+1,j) - 2*(float)img.at<uchar>(i-1,j)
														+ 1* (float)img.at<uchar>(i+1,j+1) - 1*(float)img.at<uchar>(i-1,j+1)    // revise the bug
														+ 1* (float)img.at<uchar>(i+1,j-1) - 1*(float)img.at<uchar>(i-1,j-1) )/8.0);
									}		
									
						}
						//==================================================================================
						//==================================================================================












//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Core Function ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

					//==================================================================================
					//=================  Local Search ==================================================
					//==================================================================================
					void LocalNineGridSearchMaxVal(Point& center, Point& LocalMaxP, Mat& Value)     // tested OK
					{
						int x= center.x;
						int y= center.y;

						if( x==0 || x==Value.cols-1 || y==0 || y==Value.rows-1)
							{
								cout<<"fuck! in Local Search  here is a boundary point."<<endl;
							}
	
						vector<Point>EP;

						Point P0(center.x+1, center.y); EP.push_back(P0);
						Point P1(center.x+1, center.y+1); EP.push_back(P1);
						Point P2(center.x, center.y+1); EP.push_back(P2);
						Point P3(center.x-1, center.y+1); EP.push_back(P3);
						Point P4(center.x-1, center.y);EP.push_back(P4);
						Point P5(center.x-1, center.y-1);EP.push_back(P5);
						Point P6(center.x, center.y-1);EP.push_back(P6);
						Point P7(center.x+1, center.y-1);EP.push_back(P7);

						cout<<"============================="<<endl;
						cout<<"center="<<center<<endl;
						cout<<"============================="<<endl;

						
						Point MaxP(0,0);
						float max_val=0;

						for(int i=0;i<8;i++)
						{
							int xx=EP.at(i).x;
							int yy=EP.at(i).y;

							if(max_val < Value.at<float>(yy,xx) )
								{
									max_val=Value.at<float>(yy,xx);
									cout<<"i="<<i<<"  Value.at<float>(yy,xx)="<<Value.at<float>(yy,xx)<<endl;
									MaxP=EP.at(i);
								}

						}
	
	

						if(max_val>30)   // criterion  1 : Intensity Strong Enough ?
							{
								cout<<"Value Strong enough, > 20"<<endl;
								cout<<"max_val="<<max_val<<endl<<endl;
							}


						// ========= finally we get the LocalMaxP ==========
						LocalMaxP=MaxP;
						
	
		
					}

					//==================================================================================
					//================= end of  Local Search ===========================================
					//==================================================================================








//##############################################                 core :  FindNextPoint()	################################
//##############################################################################################################################  // tested correct

	
					// IMPORTANT: Core
					void FindNextPoint(Point& seed_input,  Point& nextp_output,   Mat& Value,  float& theta_updated)          // Value and Angle:  of Float Gradient  
					{
							int x=seed_input.x;
							int y=seed_input.y;
							float theta= theta_updated +90; // +90 for unit vector orientation  , theta_updated is just the Gradient Angle

							Point2d unitVec(cos(theta*PI/180),sin(theta*PI/180));

							float stepLen=10; // pixel

							Point P_temp( cvRound(x+ unitVec.x *stepLen),   cvRound(y + unitVec.y*stepLen));
	
									

							LocalNineGridSearchMaxVal(P_temp, nextp_output, Value); // find the local maximal point as nextp_output
	
							//-------------------------------
							// up to now , we get the Next Point!
							//-------------------------------

	

					}		


// tested OK

//##############################################           end   of    core : FindNextPoint()		########################
//##############################################################################################################################








//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//##############################################                 core :  FindNextPoint_Right(),  FindNextPoint_Left()     ######
//##############################################################################################################################  

	
					// IMPORTANT: Core :----Big Jump: stepsize=10 pixel
					void FindNextPoint_Right_Big(Point& seed_input,  Point& nextp_output,   Mat& Value,  float& theta_updated)          // Value and Angle:  of Float Gradient  
					{
							int x=seed_input.x;
							int y=seed_input.y;
							float theta= theta_updated +90; // +90 for unit vector orientation  , theta_updated is just the Gradient Angle
											// we call this "Right": +90 deg.
							Point2d unitVec(cos(theta*PI/180),sin(theta*PI/180));

							float stepLen=10; // pixel

							Point P_temp( cvRound(x+ unitVec.x *stepLen),   cvRound(y + unitVec.y*stepLen));
	
									

							LocalNineGridSearchMaxVal(P_temp, nextp_output, Value); // find the local maximal point as nextp_output
	
							//-------------------------------
							// up to now , we get the Next Point!
							//-------------------------------

	

					}	

					// IMPORTANT: Core: ----Small Jump: stepsize=3 pixel
					void FindNextPoint_Right_Small(Point& seed_input,  Point& nextp_output,   Mat& Value,  float& theta_updated)          // Value and Angle:  of Float Gradient  
					{
							int x=seed_input.x;
							int y=seed_input.y;
							float theta= theta_updated +90; // +90 for unit vector orientation  , theta_updated is just the Gradient Angle
											// we call this "Right": +90 deg.
							Point2d unitVec(cos(theta*PI/180),sin(theta*PI/180));

							float stepLen=3; // pixel

							Point P_temp( cvRound(x+ unitVec.x *stepLen),   cvRound(y + unitVec.y*stepLen));
	
									

							LocalNineGridSearchMaxVal(P_temp, nextp_output, Value); // find the local maximal point as nextp_output
	
							//-------------------------------
							// up to now , we get the Next Point!
							//-------------------------------

	

					}	





					// IMPORTANT: Core  :----Big Jump: stepsize=10 pixel                // Left is Also Correct
					void FindNextPoint_Left_Big(Point& seed_input,  Point& nextp_output,   Mat& Value,  float& theta_updated)          // Value and Angle:  of Float Gradient  
					{
							int x=seed_input.x;
							int y=seed_input.y;
							float theta= theta_updated -90; // -90 for unit vector orientation  , theta_updated is just the Gradient Angle
											// we call this "Left": -90 deg.
							Point2d unitVec(cos(theta*PI/180),sin(theta*PI/180));

							float stepLen=10; // pixel

							Point P_temp( cvRound(x+ unitVec.x *stepLen),   cvRound(y + unitVec.y*stepLen));
	
									

							LocalNineGridSearchMaxVal(P_temp, nextp_output, Value); // find the local maximal point as nextp_output
	
							//-------------------------------
							// up to now , we get the Next Point!
							//-------------------------------

	

					}		
	
					// IMPORTANT: Core  :----Big Jump: stepsize=3 pixel                // Left is Also Correct
					void FindNextPoint_Left_Small(Point& seed_input,  Point& nextp_output,   Mat& Value,  float& theta_updated)          // Value and Angle:  of Float Gradient  
					{
							int x=seed_input.x;
							int y=seed_input.y;
							float theta= theta_updated -90; // -90 for unit vector orientation  , theta_updated is just the Gradient Angle
											// we call this "Left": -90 deg.
							Point2d unitVec(cos(theta*PI/180),sin(theta*PI/180));

							float stepLen=3; // pixel ; 10.29  chang from 3 to 2;

							Point P_temp( cvRound(x+ unitVec.x *stepLen),   cvRound(y + unitVec.y*stepLen));
	
									

							LocalNineGridSearchMaxVal(P_temp, nextp_output, Value); // find the local maximal point as nextp_output
	
							//-------------------------------
							// up to now , we get the Next Point!
							//-------------------------------

	

					}

// All Rihgt_Big, and Small, Left_Big and Small tested OK
//##############################################           end   of    core : FindNextPoint()		########################
//##############################################################################################################################




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




















//##############################################                 core :  JudgeSeed_On_A_Line()	################################
//##############################################################################################################################  // tested correct




						// IMPORTANT: Core  :---- Get Num   Right_Big     //also output a FinalEnd_Right Point
						int GetFindNum_Right_Big ( Point Seed, Point& FinalEnd_Right_Big, float& End_Right_Big_AngleAver, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfRight_Big=0;

							float Angle_Sum=0; // just theta sum 

							//
							float theta_updated=Angle.at<float>(Seed.y,Seed.x);       //initialize theta_updated; 

							float theta_last=Angle.at<float>(Seed.y,Seed.x);

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=Seed;    
							Point P_Next(0,0);

							for(int i=0;i<20;i++)
							{
								FindNextPoint_Right_Big(P, P_Next,  diff_long_ver_float,  theta_updated);

								theta_last=theta_updated;			// preserve theta_last
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
cout<<"----------------fuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuck"<<endl;
cout<<"		theta_updated= "<<theta_updated<<endl;	
								if( abs(theta_updated -theta_last) < 10  ||  abs(theta_updated -theta_last -360) < 10 ||  abs(theta_updated -theta_last +360) < 10 ) 
					
										// consider +180 and -180 case
								{	

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	



										numOfRight_Big++;


										// Angle Range things. Not small deal
										// Serious Bug Found and Revised
										if(abs(theta_updated -theta_last) < 10 )  // normal case
										{	
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360 ;
								
										}											
										


										if(abs(theta_updated -theta_last -360) < 10)              // 180  alternate -180,  all change to, positive  0--360
										{	
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360;                      // theta updated is 177,178 around.i.e.
										}


										if(abs(theta_updated -theta_last +360) < 10)              // 180  alternate -180,  all change to, positive  0--360
										{
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360;            // now theta_updated is  -177, 178 around
										}
										





										P=P_Next;  // iterate									
						
								}		

						
								else

									break;
					
							}

							// final end point					
							FinalEnd_Right_Big = P;


							// aver angle theta

							// big bug revised.  we need to chang AngleAver from 0~360  [0,360)   back to -180  ~ 180  (-180,180]

							End_Right_Big_AngleAver= Angle_Sum/numOfRight_Big;

							if(End_Right_Big_AngleAver>=0 && End_Right_Big_AngleAver<360)
								{
									cout<<"End_Right_Big_AngleAver is in correct Range as expected up to now."<<endl;


									cout<<endl<<endl<<endl;
									cout<<"*************************"<<endl;
									cout<<"	End_Right_Big_AngleAver="<<End_Right_Big_AngleAver<<endl;
									cout<<endl<<endl<<endl;

									if(End_Right_Big_AngleAver>180)
										End_Right_Big_AngleAver= End_Right_Big_AngleAver-360;      // key revised



									cout<<endl<<endl<<endl;
									cout<<"*************************"<<endl;
									cout<<"	End_Right_Big_AngleAver="<<End_Right_Big_AngleAver<<endl;
									cout<<endl<<endl<<endl;
								
								}

							else

								{
									cout<<"Error in Ang.  End_Right_Big_AngleAver is NOT in correct Range as expected up to now."<<endl;
								}

							return numOfRight_Big;

							

						}   
			
						// end of GetFindNum_Right_Big()
						//----------------------------------------------------------






						// IMPORTANT: Core  :---- Get Num   Right_Big     //also output a FinalEnd_Right Point
					int GetFindNum_Right_Small ( Point Seed, Point& FinalEnd_Right_Small,  Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfRight_Small=0;

							

							//
							float theta_updated=Angle.at<float>(Seed.y,Seed.x);       //initialize theta_updated; 

							float theta_last=0;

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=Seed;    
							Point P_Next(0,0);

							for(int i=0;i<20;i++)
							{
								FindNextPoint_Right_Small(P, P_Next,  diff_long_ver_float,  theta_updated);

								theta_last=theta_updated;			// preserve theta_last
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
			
								if( abs(theta_updated -theta_last) < 10  ||  abs(theta_updated -theta_last -360) < 10 ||  abs(theta_updated -theta_last +360) < 10 ) 
					
										// consider +180 and -180 case
								{	

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	



										numOfRight_Small++;

										
										
										P=P_Next;  // iterate									
						
								}		

						
								else

									break;
					
							}

							// final end point					
							FinalEnd_Right_Small = P;

							

							return numOfRight_Small;

							

						}   
			
						// end of GetFindNum_Right_Big()
						//----------------------------------------------------------







				
						// IMPORTANT: Core  :---- Get Num   Right_Big
						int GetFindNum_Left_Big ( Point Seed, Point& FinalEnd_Left_Big, float& End_Left_Big_AngleAver, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfLeft_Big=0;

							float Angle_Sum=0; // just theta sum 

							//
							float theta_updated=Angle.at<float>(Seed.y,Seed.x);       //initialize theta_updated; 

							float theta_last=0;

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=Seed;    
							Point P_Next(0,0);

							for(int i=0;i<20;i++)
							{
								FindNextPoint_Left_Big(P, P_Next,  diff_long_ver_float,  theta_updated);

								theta_last=theta_updated;			// preserve theta_last
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
			
								if( abs(theta_updated -theta_last) < 10  ||  abs(theta_updated -theta_last -360) < 10 ||  abs(theta_updated -theta_last +360) < 10 ) 
					
										// consider +180 and -180 case
								{	

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	



										numOfLeft_Big++;
				

									// Serious Bug Found and Revised
										if(abs(theta_updated -theta_last) < 10 )  // normal case
										{	
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360 ;
								
										}											
										


										if(abs(theta_updated -theta_last -360) < 10)              // 180  alternate -180,  all change to, positive  0--360
										{	
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360;                      // theta updated is 177,178 around.i.e.
										}


										if(abs(theta_updated -theta_last +360) < 10)              // 180  alternate -180,  all change to, positive  0--360
										{
											if(theta_updated>0)                                   // big bug revised
												Angle_Sum+= theta_updated;
											else
												Angle_Sum+= theta_updated + 360;            // now theta_updated is  -177, 178 around
										}

										cout<<"*************"<<endl;
										cout<<"Angle_Sum="<<Angle_Sum<<endl;
										cout<<"*************"<<endl;

										P=P_Next;  // iterate	

								
						
								}		

						
								else

									break;
					
							}

							// final end point
							FinalEnd_Left_Big= P;

							// aver angle theta 

							// big bug revised.  we need to chang AngleAver from 0~360  [0,360)   back to -180  ~ 180  (-180,180]

							End_Left_Big_AngleAver= Angle_Sum/numOfLeft_Big;

							if(End_Left_Big_AngleAver>=0 && End_Left_Big_AngleAver<360)
								{
									cout<<"End_Left_Big_AngleAver is in correct Range as expected up to now."<<endl;

									cout<<endl<<endl<<endl;
									cout<<"*************************"<<endl;
									cout<<"	End_Left_Big_AngleAver="<<End_Left_Big_AngleAver<<endl;
									cout<<endl<<endl<<endl;


									if(End_Left_Big_AngleAver>180)
										End_Left_Big_AngleAver= End_Left_Big_AngleAver-360;      // key revised

									cout<<endl<<endl<<endl;
									cout<<"*************************"<<endl;
									cout<<"	End_Left_Big_AngleAver="<<End_Left_Big_AngleAver<<endl;
									cout<<endl<<endl<<endl;
											



								}

							else

								{
									cout<<"Error in Ang.  End_Left_Big_AngleAver is NOT in correct Range as expected up to now."<<endl;
								}

								

							return numOfLeft_Big;

							

						}   
			
						// end of GetFindNum_Left_Big()
						//----------------------------------------------------------









						// IMPORTANT: Core  :---- Get Num   Left_Small
						int GetFindNum_Left_Small ( Point Seed, Point& FinalEnd_Left_Small, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfLeft_Small=0;

							//
							float theta_updated=Angle.at<float>(Seed.y,Seed.x);       //initialize theta_updated; 

							float theta_last=0;

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=Seed;    
							Point P_Next(0,0);

							for(int i=0;i<20;i++)
							{
								FindNextPoint_Left_Small(P, P_Next,  diff_long_ver_float,  theta_updated);

								theta_last=theta_updated;			// preserve theta_last
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
			
								if( abs(theta_updated -theta_last) < 10  ||  abs(theta_updated -theta_last -360) < 10 ||  abs(theta_updated -theta_last +360) < 10 ) 
					
										// consider +180 and -180 case    
								{	

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	



										numOfLeft_Small++;
										
										P=P_Next;  // iterate									
						
								}		

						
								else

									break;
					
							}

							// final end point
							FinalEnd_Left_Small= P;

							return numOfLeft_Small;

							

						}   
			
						// end of GetFindNum_Left_Small()
						//----------------------------------------------------------











						bool  JudgeSeed_On_A_Line(Point Seed,  Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver )
															    // here diff_long_ver only for draw, will omitted				
						{
							// target bool
							bool YesOrNo=0;


							// continuous successful trace steps nums as criterion
							int numOfRight_Big=0, numOfRight_Small=0;
							int numOfLeft_Big=0, numOfLeft_Small=0;
							int numOfBig=0, numOfSmall=0;

							Point FinalEnd_Left_Big=Point(0,0);
							Point FinalEnd_Right_Big=Point(0,0);
							
		
							// angle
							float End_Right_Big_AngleAver;  // just for match the para, none use 
				
							// Get Num :
							// Num_Right_Big  and Final end
							
							numOfRight_Big = GetFindNum_Right_Big ( Seed, FinalEnd_Right_Big, End_Right_Big_AngleAver, diff_long_ver_float, Angle,  diff_long_ver); 

								cout<<endl<<endl<<"^^^^^^^^^^^^^^"<<endl;
								cout<<"numOfRight_Big for Point "<<Seed<< " = "<<numOfRight_Big<<endl;
								cout<<"FinalEnd_Right_Big="<<FinalEnd_Right_Big<<endl;
								cout<<"^^^^^^^^^^^^^^^^"<<endl<<endl;

							
							// angle:
							float End_Left_Big_AngleAver;  // just for match the para, none use
							// Num_Right_Big  and Final end
							numOfLeft_Big = GetFindNum_Left_Big ( Seed, FinalEnd_Left_Big, End_Left_Big_AngleAver, diff_long_ver_float, Angle,  diff_long_ver); 

								cout<<endl<<endl<<"^^^^^^^^^^^^^^"<<endl;
								cout<<"numOfLeft_Big for Point "<<Seed<< " = "<<numOfLeft_Big<<endl;
								cout<<"FinalEnd_Left_Big="<<FinalEnd_Left_Big<<endl;
								cout<<"^^^^^^^^^^^^^^^^"<<endl<<endl;


							// sum 
							numOfBig= numOfRight_Big + numOfLeft_Big;


							// Criterion							
							if(numOfBig >= 8)  // 10 is too strong since the right L output only 12, here changes to 8
								{
									cout<<endl<<endl<<"-----------------------------"<<endl;
									cout<<"Hah Yes, it is on A Line !     with numOfBig="<< numOfBig<<endl<<endl;
									YesOrNo= true;

								}	

							else
								{
									cout<<endl<<endl<<" -----------------------------"<<endl;
									cout<<"Oh NO, it is  not on A Line !     with numOfBig="<< numOfBig<<endl<<endl;
									YesOrNo= false;
								}
							
							



							return YesOrNo;
		
						}

						// end of JudgeSeed_On_A_Line()
						//----------------------------------------------------------




// revised : add final end, still test OK for Big.   NO test for Small.

// TEST RESULT: up to now, all the Judge are tested correct!

//##############################################           end   of    core : JudgeSeed_On_A_Line()	########################
//##############################################################################################################################













//int GetFindNum_Left_Small ( Point Seed, Point& FinalEnd_Left_Small, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)





//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//##############################################                 core :  TwoEndFinding()     ######
//##############################################################################################################################  
			
	

						//Left End
						Point LeftEndFinding (Point Seed,  float& End_Left_AngleAver,  int& numOfLeft_Big,   Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							
							float End_Left_Big_AngleAver;
						
						

							// first get Left_Big jump end;
							Point FinalEnd_Left_Big=Point(0,0);
							numOfLeft_Big=
							GetFindNum_Left_Big ( Seed, FinalEnd_Left_Big, End_Left_Big_AngleAver, diff_long_ver_float, Angle, diff_long_ver);

							// for AngleAver, only consider the Big Jump Average
			
							End_Left_AngleAver=End_Left_Big_AngleAver;    //%%%%%%%%%%%%%%%%%%%%%%% Angle Here


							//second , small jump to confine
							Point Seed_After_BigJump = FinalEnd_Left_Big;  // initialise for small jump
							
							Point FinalEnd_Left_Small=Point(0,0);
							
							GetFindNum_Left_Small ( Seed_After_BigJump, FinalEnd_Left_Small, diff_long_ver_float, Angle, diff_long_ver);


							// Key Key Key !!!
							
							// Key Step: One More Find
							// Reason:  	3 pixel not close enough
							float theta_updated= Angle.at<float>(FinalEnd_Left_Small.y,FinalEnd_Left_Small.x);
	
							Point One_Step_Further(0,0);

							FindNextPoint_Left_Small(FinalEnd_Left_Small, One_Step_Further,  diff_long_ver_float,  theta_updated);


							// After I Add this Key step further, the 3 pixel missing is recovered. And the corner is perfect!						


							return One_Step_Further;
	
						}


						//Right End
						Point RightEndFinding (Point Seed, float& End_Right_AngleAver, int& numOfRight_Big,    Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{

							float End_Right_Big_AngleAver;


							// first get Right_Big jump end;
							Point FinalEnd_Right_Big=Point(0,0);
							numOfRight_Big=
							GetFindNum_Right_Big ( Seed, FinalEnd_Right_Big, End_Right_Big_AngleAver, diff_long_ver_float, Angle, diff_long_ver);

							// for AngleAver, only consider the Big Jump Average
							End_Right_AngleAver=End_Right_Big_AngleAver;    //%%%%%%%%%%%%%%%%%%%%%%% Angle Here


							//second , small jump to confine
							Point Seed_After_BigJump = FinalEnd_Right_Big;  // initiallise for small jump
							
							Point FinalEnd_Right_Small=Point(0,0);

							GetFindNum_Right_Small ( Seed_After_BigJump, FinalEnd_Right_Small, diff_long_ver_float, Angle, diff_long_ver);


							// Key Key Key !!!
							
							// Key Step: One More Find
							// Reason:  	3 pixel not close enough
							float theta_updated= Angle.at<float>(FinalEnd_Right_Small.y,FinalEnd_Right_Small.x);
	
							Point One_Step_Further(0,0);

							FindNextPoint_Right_Small(FinalEnd_Right_Small, One_Step_Further,  diff_long_ver_float,  theta_updated);


							// After I Add this Key step further, the 3 pixel missing is recovered. And the corner is perfect!						


							return One_Step_Further;
	
						}







//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// dollar func

						//input: one seed
						//output: two ends

						// precondition: it is on a line

					void TwoEndFinding_And_AngleAver(Point Seed, Point& End_Left, Point& End_Right,  float& AngleAver,   Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver )
	
						{
							
							float  End_Left_AngleAver=0;
							float  End_Right_AngleAver=0;

							int numOfLeft_Big;
							int numOfRight_Big;
							// end points

							End_Left= LeftEndFinding(Seed,  End_Left_AngleAver, numOfLeft_Big,   diff_long_ver_float, Angle, diff_long_ver);
	
											
							End_Right= RightEndFinding(Seed, End_Right_AngleAver, numOfRight_Big,  diff_long_ver_float, Angle, diff_long_ver);
							
							cout<<"-------------"<<endl;	
							cout<<"End_Left="<<End_Left<<endl;

							cout<<"-------------"<<endl;	
							cout<<"End_Right="<<End_Right<<endl;


							// angle

							if(    (End_Left_AngleAver * End_Right_AngleAver < 0)  // one pos one neg

								&& abs(End_Left_AngleAver - End_Right_AngleAver) > 180  )

								{

									// transform to 0~360 to handle
									if(End_Left_AngleAver>0)  // change to 0~360 to handle this weired case

										End_Right_AngleAver =End_Right_AngleAver+360;

									else

										End_Left_AngleAver =End_Left_AngleAver+360;
										




									// calculate the weighted average
							
									AngleAver= (numOfLeft_Big*End_Left_AngleAver + numOfRight_Big* End_Right_AngleAver)/(numOfLeft_Big+numOfRight_Big) ; 
		

										// big BUG :1. here both angle in range [-180, 180],

										// if they both positive, or both negative, are OK.
										// but when one is 178 the other -179. it will not be 180 or -180 but 0 !!!!

										// Notice : for -9 and +7, it is OK though one positive one negative.

										// Bug 2: here we only half not weighted aver. Revised.
				
									// notice this AngleAver is [0,360]






									// transform back to [-180, 180]

									if(AngleAver>180)
										AngleAver=AngleAver-360;
							
									

								}



							else     // normal case


								AngleAver= (numOfLeft_Big*End_Left_AngleAver + numOfRight_Big* End_Right_AngleAver)/(numOfLeft_Big+numOfRight_Big) ; 
	


//------------------------------------------up to now, we handle the last Weird Angle------------------------------------------------

							cout<<"-------------"<<endl;	
							cout<<"AngleAver="<<AngleAver<<endl;
						
						}

			




				//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
				//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//gold

//input: just a seed

//output: total line 

			
						// Line Para Assign
						// 
						void LineAssign(Point Seed, LineSeg& ThisLine,  Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver )

						{
			
							Point End_Left(0,0);
							Point End_Right(0,0);
							float AngleAver=0;
							
							
							// calculate all line para
							TwoEndFinding_And_AngleAver(Seed, End_Left, End_Right,  AngleAver,  diff_long_ver_float, Angle, diff_long_ver );

							
							// assign This Line all para
									
								ThisLine.Seed=Seed;
								ThisLine.LeftEnd=End_Left;
								ThisLine.RightEnd=End_Right;
								ThisLine.Average_Theta= AngleAver<180 ? AngleAver: AngleAver-360 ;   //back to -180 to 180    //Tested, All correct

								ThisLine.Length= distanceOfTwoPoints(ThisLine.LeftEnd, ThisLine.RightEnd);
								ThisLine.StepLen= ThisLine.Length /12;
	
								printLineSeg(ThisLine);
 
							//end of assign Line			// Tested, Correct
							
						}


		

// end of dollar func
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$



//##############################################           end   of    core : TwoEndFinding()	########################
//##############################################################################################################################










































							
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   LE       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%












//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//##############################################                 core :  Turnable_LE_Check_And_NewSeedGet()               ######
//##############################################################################################################################  




// just travel left Num steps , return the final point

						Point Travel_Left_Small_Many_Steps(Point StartP, int NumOfSteps, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver) 
			
						{
							

							//
							float theta_updated=Angle.at<float>(StartP.y, StartP.x);       //initialize theta_updated; 

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=StartP;    
							Point P_Next(0,0);

							for(int i=0;i<NumOfSteps;i++)
							{
								FindNextPoint_Left_Small(P, P_Next,  diff_long_ver_float,  theta_updated);
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
			
								

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	
										
								P=P_Next;  // iterate									
						
								
					
							}

							// final end point
							Point Travel_End= P;

							return Travel_End;


						}























//3_check: 1. orthogonal;  2. is a Line; 3. Length close
						
						bool AfterTurn_LE_3_Check(LineSeg& ThisLine, Point& New_Seed,  Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)

						{
							// check 1: orthogonal
							// abs( Ang - Ang - 90) < 10

							float Ang_New_Seed=  Angle.at<float>(New_Seed.y, New_Seed.x);

							float Ang_ThisLine=  ThisLine.Average_Theta;

							bool check_1;

							if(  	abs (abs(Ang_New_Seed - Ang_ThisLine) - 90) < 10    // normal case
							   ||   abs (abs(Ang_New_Seed - Ang_ThisLine -360) -90) <10   // New 178, This -89. case
							   ||   abs (abs(Ang_New_Seed - Ang_ThisLine +360) -90) <10)  // New -90, This 178. case

								{
									check_1 = true;
								}
										 
							else
								{	
									check_1 = false;
									return false;      // one check fail, all fail
								}
							
			//--------------------------------  up to now, we know whether orthogonal  -----------------------------------------------------------------------





							// check 2: is a Line ?

							bool check_2=   JudgeSeed_On_A_Line( New_Seed,  diff_long_ver_float, Angle, diff_long_ver );

							if (check_2==false)
		
								return false;

			//--------------------------------  up to now, we know whether there is a line  -----------------------------------------------------------------------


				
							// check 3:  abs(Length -Length) < 30

							bool check_3;

							LineSeg OtherLine={Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};

							LineAssign(New_Seed, OtherLine,  diff_long_ver_float,  Angle,  diff_long_ver );


							if( abs (  ThisLine.Length -  OtherLine.Length )  < 30   )

								check_3=true;

							else

								{
									check_3=false;

									return false;
								}

							

			//--------------------------------  up to now, we know whether the length close  -----------------------------------------------------------------------


					

						// up to now, it means it did not return, in other words, it passes 3 check. return true.


				
							return true;    
							
					
						}























//##############################################################################################################################
// method: from the end point, small jump left 10 steps, call this position as New_Seed;
//         check abs(New_Seed's theta  -  ThisLine's theta -90) < 10, that is orthogonal. go to next. else return false;
//         Next, for this new seed, JudgeOnLine  -->  TwoEnd  ---> Line Assign


						bool Turnable_LE_Check_And_OtherLine_Get ( LineSeg& ThisLine,  LineSeg& OtherLine,   Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{	


							//$ first, go 10 steps;

							//input: One point 
							//output: point after 10 small steps left

							Point LeftEnd= ThisLine.LeftEnd; // start point of turn
							
							Point New_Seed(0,0);

							New_Seed=Travel_Left_Small_Many_Steps(LeftEnd, 10,  diff_long_ver_float,  Angle,  diff_long_ver);

//--------------------------------------------------    up to now, we get the point 10 small steps away   --------------------------------------------------------------------------------



cout<<"fuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the angle!"<<endl;


							//$ second, totally check  the new seed, and the other line
												

							// Now check: 1. orthogonal;  2. is a Line; 3. Length close
							bool Turnable_Check= AfterTurn_LE_3_Check(ThisLine, New_Seed,  diff_long_ver_float, Angle, diff_long_ver);

//-----------------------   up to now, we know whether it is turnable, i mean  whether there is an equal perfect line waiting there ----------------------------------------------------------





							//$ finally, get the other line;


							if(Turnable_Check == true)
								{

									//this Key
									LineAssign(New_Seed, OtherLine,  diff_long_ver_float,  Angle,  diff_long_ver );
			
									return true;

//---------------------------------------   up to now,  we get the other Line.  Our final target! ----------------------------------------------------------------------------------------------
 								

								}

							else
								{
									cout<<"Turn around Left End fail!"<<endl; 
									return false;
								}
									
					
						}		


//##############################################           end   of    core : 	Turnable_LE_Check_And_NewSeedGet() 	  ######
//##############################################################################################################################




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------












						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  end of LE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%











































						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   RE       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		







//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//##############################################                 core :  Turnable_RE_Check_And_NewSeedGet()              ######
//##############################################################################################################################  




// just travel left Num steps , return the final point

						Point Travel_Right_Small_Many_Steps(Point StartP, int NumOfSteps, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver) 
			
						{
							

							//
							float theta_updated=Angle.at<float>(StartP.y, StartP.x);       //initialize theta_updated; 

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=StartP;    
							Point P_Next(0,0);

							for(int i=0;i<NumOfSteps;i++)
							{
								FindNextPoint_Right_Small(P, P_Next,  diff_long_ver_float,  theta_updated);
					
								theta_updated= Angle.at<float>(P_Next.y,P_Next.x);       // update theta_updated
		
			
								

											//=========draw P_Next out on 
										 	circle(diff_long_ver_copy, P_Next, 3, Scalar(255), 1, 8, 0);
				
											cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
											    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

											imshow("diff_long_ver_copy_JudgeOnLine",diff_long_ver_copy);
											imwrite("diff_long_ver_copy_JudgeOnLine.jpg",diff_long_ver_copy);
											waitKey(0);
											//==========end of draw P out
	
										
								P=P_Next;  // iterate									
						
								
					
							}

							// final end point
							Point Travel_End= P;

							return Travel_End;


						}























//3_check: 1. orthogonal;  2. is a Line; 3. Length close
						
						bool AfterTurn_RE_3_Check(LineSeg& ThisLine, Point& New_Seed,  Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)

						{
							// check 1: orthogonal
							// abs( Ang - Ang - 90) < 10

							float Ang_New_Seed=  Angle.at<float>(New_Seed.y, New_Seed.x);

							float Ang_ThisLine=  ThisLine.Average_Theta;

							bool check_1;

							if(  	abs (abs(Ang_New_Seed - Ang_ThisLine) - 90) < 10    // normal case
							   ||   abs (abs(Ang_New_Seed - Ang_ThisLine -360) -90) <10   // New 178, This -89. case
							   ||   abs (abs(Ang_New_Seed - Ang_ThisLine +360) -90) <10)  // New -90, This 178. case

								{
									check_1 = true;
								}
										 
							else
								{	
									check_1 = false;
									return false;      // one check fail, all fail
								}
							
			//--------------------------------  up to now, we know whether orthogonal  -----------------------------------------------------------------------





							// check 2: is a Line ?

							bool check_2=   JudgeSeed_On_A_Line( New_Seed,  diff_long_ver_float, Angle, diff_long_ver );

							if (check_2==false)
		
								return false;

			//--------------------------------  up to now, we know whether there is a line  -----------------------------------------------------------------------


				
							// check 3:  abs(Length -Length) < 30

							bool check_3;

							LineSeg OtherLine={Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};

							LineAssign(New_Seed, OtherLine,  diff_long_ver_float,  Angle,  diff_long_ver );


							if( abs (  ThisLine.Length -  OtherLine.Length )  < 30   )

								check_3=true;

							else

								{
									check_3=false;

									return false;
								}

							

			//--------------------------------  up to now, we know whether the length close  -----------------------------------------------------------------------


					

						// up to now, it means it did not return, in other words, it passes 3 check. return true.


				
							return true;    
							
					
						}























//##############################################################################################################################
// method: from the end point, small jump right 10 steps, call this position as New_Seed;
//         check abs(New_Seed's theta  -  ThisLine's theta -90) < 10, that is orthogonal. go to next. else return false;
//         Next, for this new seed, JudgeOnLine  -->  TwoEnd  ---> Line Assign


						bool Turnable_RE_Check_And_OtherLine_Get ( LineSeg& ThisLine,  LineSeg& OtherLine,   Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{	


							//$ first, go 10 steps;

							//input: One point 
							//output: point after 10 small steps left

							Point RightEnd= ThisLine.RightEnd; // start point of turn
							
							Point New_Seed(0,0);

							New_Seed=Travel_Right_Small_Many_Steps(RightEnd, 10,  diff_long_ver_float,  Angle,  diff_long_ver);

//--------------------------------------------------    up to now, we get the point 10 small steps away   --------------------------------------------------------------------------------



cout<<"fuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the anglefuck fuck fuck the angle!"<<endl;


							//$ second, totally check  the new seed, and the other line
												

							// Now check: 1. orthogonal;  2. is a Line; 3. Length close
							bool Turnable_Check= AfterTurn_RE_3_Check(ThisLine, New_Seed,  diff_long_ver_float, Angle, diff_long_ver);

//-----------------------   up to now, we know whether it is turnable, i mean  whether there is an equal perfect line waiting there ----------------------------------------------------------





							//$ finally, get the other line;


							if(Turnable_Check == true)
								{

									//this Key
									LineAssign(New_Seed, OtherLine,  diff_long_ver_float,  Angle,  diff_long_ver );
			
									return true;

//---------------------------------------   up to now,  we get the other Line.  Our final target! ----------------------------------------------------------------------------------------------
 								

								}

							else
								{
									cout<<"Turn around Right End fail!"<<endl; 
									return false;
								}
									
					
						}		


//##############################################           end   of    core : 	Turnable_RE_Check_And_NewSeedGet() 	  ######
//##############################################################################################################################




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------








						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  end of RE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





















// all writing of RE turn is finished;  Need Test!  Left is OK , while right ?


// Tested.   All RE part is correct.  11-1


























//##############################################                 core :   OtherLineFind()	                          ######
//##############################################################################################################################  

						bool OtherLineFind(LineSeg& ThisLine, LineSeg& OtherLine, bool& Indicate_Left_OR_Right, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
													// bug revised: bool& , not bool						
						{

							//Line Candidate
							LineSeg OtherLine_LE={Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};
							LineSeg OtherLine_RE={Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};

							bool Turnable_LE=0;
							bool Turnable_RE=0;

							//output is otherLine
							

							// find from the left point
							
							Turnable_LE= Turnable_LE_Check_And_OtherLine_Get ( ThisLine,  OtherLine_LE,   diff_long_ver_float, Angle, diff_long_ver);


							// if find from left  fail, try right	


							if(Turnable_LE==1)
							{							
											cout<<endl<<endl<<"--------------------------"<<endl;
											cout<<"Other Line found from Left End."<<endl;
											cout<<endl<<endl;


									OtherLine=OtherLine_LE; // finally we got other line.

									Indicate_Left_OR_Right =1;  // indicate Left turn suc.  tell the following Determine_10Rits func.

									return true;

									


							}		

							else

							{			
									// find from the right point 
									
									Turnable_RE= Turnable_RE_Check_And_OtherLine_Get ( ThisLine,  OtherLine_RE,   diff_long_ver_float, Angle, diff_long_ver);
						
									if(Turnable_RE==1)

										{
											
												cout<<endl<<endl<<"--------------------------"<<endl;
												cout<<"Other Line found from Right End."<<endl;
												cout<<endl<<endl;



											OtherLine=OtherLine_RE; // finally we got other line.
	
											Indicate_Left_OR_Right =0;  // indicate Left turn suc.  tell the following Determine_10Rits func.

											return true;

					

										}
		
							}

							// to here, RE and LE both fail, cannot turn.
		
							return false;



						}		









//##############################################           end   of    core : OtherLineFind()         			  ######
//##############################################################################################################################









//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------























//##############################################                 core :   Determine_TenRits()	                          ######
//##############################################################################################################################  





						void Determine_TenRits( Ten_Rits& Ten_Rits_Gold_OutPut , LineSeg& ThisLine, LineSeg& OtherLine, bool Indicate_Left_OR_Right)

						{
							// 1, first, judge who is Line_I, who is Line_II
			
							LineSeg Line_I= {Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};
							LineSeg Line_II= {Point(0,0), Point(0,0), Point(0,0), 0, 0, 0};

												
		

							if( Indicate_Left_OR_Right == 1)

									{
										Line_I= ThisLine;
										Line_II= OtherLine;
									}


							else
									{
										Line_I= OtherLine;
										Line_II= ThisLine;
									}	

				//--------------------
				//--------------------- up to now, we got Line_I and Line_II, all information  ------------------------------------------------------




						
							//2. second, set the origin.  

							Point Origin(0,0);

							//check two end is near, just for sure. 

							if(  distanceOfTwoPoints(Line_I.LeftEnd, Line_II.RightEnd)  < 6 )
			
								{
									cout<<endl<<endl<<" Sure, the two near ends are near.  as expected. "<<endl<<"----------------------------"<<endl<<endl;
	
								}		

							Origin= (Line_I.LeftEnd + Line_II.RightEnd)*0.5;


				//--------------------
				//--------------------- up to now, we got origin------------------------------------------------------
								




							//3. third, set the StepVec1, StepVec2
	
							Point2f StepVec1(0,0), StepVec2(0,0);

							// check two StepLen are near
				
							if(  abs(Line_I.StepLen - Line_II.StepLen)  < 3 )
			
								{
									cout<<endl<<endl<<" Sure, the two near StepLen are near.  as expected. "<<endl<<"----------------------------"<<endl<<endl;
	
									cout<<"abs(Line_I.StepLen - Line_II.StepLen)="<<abs(Line_I.StepLen - Line_II.StepLen)<<endl;
										
								}	
							

							//float StepLen= (Line_I.StepLen + Line_II.StepLen) *0.5;   // NOTICE HERE: change to saperate case.  since distortion.

							float StepLen_1= Line_I.StepLen;
							float StepLen_2= Line_II.StepLen;


				//--------------------
				//--------------------- up to now, we got StepLen------------------------------------------------------

						StepVec1= Point2f(   StepLen_1* cos( (Line_I.Average_Theta +90  ) * PI/180 )      ,     StepLen_1* sin( (Line_I.Average_Theta +90 ) * PI/180 )     );
						StepVec2= Point2f(   StepLen_2* cos( (Line_II.Average_Theta -90 ) * PI/180 )      ,     StepLen_2* sin( (Line_II.Average_Theta -90 ) * PI/180 )     );

							//big bug revised. not Point() but Point2f()
							
				//--------------------
				//--------------------- up to now, we got StepVec1, StepVec2------------------------------------------------------



							




cout<<"-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
cout<<"$$$$$$$$$$$$$$$$$$$$$"<<endl;                           

cout<<"					                                            WE FINALLY GOT THE SIX RITS.                                                                            "<<endl;

cout<<"$$$$$$$$$$$$$$$$$$$$$"<<endl;   
cout<<"-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;


							// display 6 Rits
							cout<<endl<<endl<<endl;
			
							cout<<"Origin="<<Origin<<endl;
							cout<<"StepVec1="<<StepVec1<<endl;
							cout<<"StepVec2="<<StepVec2<<endl;

							cout<<endl<<endl<<endl;




							//4. fourth, set A and B.			


							Point A= Line_I.RightEnd;
							Point B= Line_II.LeftEnd;
						

				//--------------------
				//--------------------- up to now, we got A and B.   10 Rits  Are All gotten.------------------------------------------------------






							//5. Fifth, Assign the 10 Rits

							Ten_Rits_Gold_OutPut.Origin= Origin;
							Ten_Rits_Gold_OutPut.StepVec1= StepVec1;
							Ten_Rits_Gold_OutPut.StepVec2= StepVec2;
		

							Ten_Rits_Gold_OutPut.A= A;
							Ten_Rits_Gold_OutPut.B= B;

							// assign finished
 




cout<<"-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
cout<<"$$$$$$$$$$$$$$$$$$$$$"<<endl;                           

cout<<"					                                            WE FINALLY GOT THE TEN RITS.                                                                            "<<endl;

cout<<"$$$$$$$$$$$$$$$$$$$$$"<<endl;   
cout<<"-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;

			
							cout<<endl<<endl<<endl;
							cout<<"A="<<A<<endl;
							cout<<"B="<<B<<endl;
							cout<<endl<<endl<<endl;			






						}
					








//##############################################           end   of    core : Determine_TenRits()         		  ######
//##############################################################################################################################






//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   End of Core Function   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------





















//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    Final L    		  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//input: One Seed

//Output: 6 Rits --- Origin, StepVec1, StepVec2
 




					// bool is sign of whether the Seed is on a L
					//
					bool L( Point Seed,  Ten_Rits& Ten_Rits_Gold_OutPut,       Mat& diff_long_ver_float, Mat& Angle,    Mat& diff_long_ver)
					{
			
						// First, it is on a Line ? 

						bool OnLineOrNot= JudgeSeed_On_A_Line(Seed, diff_long_ver_float, Angle,  diff_long_ver );


							cout<<endl<<endl<<"--------------------"<<endl;		
							cout<<"OnLineOrNot="<<OnLineOrNot<<endl;
							cout<<endl<<"-----------------"<<endl;	

						// end First

					//--------------------------------------------------------------


						// Second,  Assign this Line para                        // Angle Tested Correct !!


						LineSeg ThisLine={Point(0,0),Point(0,0),Point(0,0),0, 0, 0};

						LineSeg OtherLine={Point(0,0),Point(0,0),Point(0,0),0, 0, 0};

						// pass the line check
						if( OnLineOrNot == 1)

							{
								// initiallize para
								Point End_Left(0,0);
								Point End_Right(0,0);
								float AngleAver=0;		
		
								// calculate all paras
								TwoEndFinding_And_AngleAver(Seed, End_Left, End_Right,  AngleAver, diff_long_ver_float, Angle, diff_long_ver ); // Tested , all correct

								// assign This Line all para
									
								ThisLine.Seed=Seed;
								ThisLine.LeftEnd=End_Left;
								ThisLine.RightEnd=End_Right;
								ThisLine.Average_Theta= AngleAver<180 ? AngleAver: AngleAver-360 ;   //back to -180 to 180    //Tested, All correct

								ThisLine.Length= distanceOfTwoPoints(ThisLine.LeftEnd, ThisLine.RightEnd);
								ThisLine.StepLen= ThisLine.Length /12;
	
								printLineSeg(ThisLine);
 
								//end of assign Line			// Tested, Correct


								//--------------------------------------------------------------
								//-------------------------Up to now, we have gotten One Line's All information ------------------------
								//--------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
								
								//--------------------------------------------------------------
								//-------------------------Check the Other Line Existence and Get it------------------------
								//--------------------------------------------------------------	
								
		
								bool Indicate_Left_OR_Right; // 1 for Left Turn Success;  0 for Right Turn Success. 

											     // Indicate the two lines which is which, which is Line 1 and which is Line 2.		
					

								bool OtherLineFoundOrNot=OtherLineFind(ThisLine, OtherLine,  Indicate_Left_OR_Right, diff_long_ver_float, Angle, diff_long_ver);


								printLineSeg(ThisLine);


			
								//pass two line check
								// if L OK
								if(OtherLineFoundOrNot == true )
	
									{
												cout<<endl;
												cout<<endl;
												cout<<"Yes, can Turn! "<<endl;
												cout<<endl;	
												cout<<endl;
	
												cout<<"Indicate="<<Indicate_Left_OR_Right<<endl;  // test correct
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
										//final gold Ten Rits here
										//Determine_TenRits

										Determine_TenRits(Ten_Rits_Gold_OutPut , ThisLine, OtherLine, Indicate_Left_OR_Right);

		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


										return true;
									}


								// pass one line fail the other
								// if L not OK	
								else
									{

											cout<<endl<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;	
											cout<<"No! This Point "<<Seed<<" is only On a LIne, Not L."<<endl;
											cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;	
											cout<<endl<<endl;
	
										return false;


									}

								
			
							}



						// did not pass the sigle line check
						else
							
							{	
								cout<<endl<<endl;
								cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;	
								cout<<"NO! This Point "<<Seed<<" is not On a L, even not a single line."<<endl;
								cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;	
								cout<<endl<<endl;				
											
								return 0;
							}





						return 0; // temp


					}

        	





//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   End of Final L         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



















//=========================================================================================
//==========================================================================================for RS
//==========================================================================================


void reorder(Mat& raw, uchar* zo)
{
	Mat temp(raw.rows,raw.cols,CV_8UC1, Scalar(0) );

	for(int i=0;i<raw.rows;i++)
		for(int j=0;j<raw.cols;j++)
		{
			{
			if(raw.at<uchar>(i,j)==1)
				temp.at<uchar>(10-1-i,j)=16;
			else
				temp.at<uchar>(10-1-i,j)=23;
			}	

			zo[(10-1-i)*raw.rows+j]=temp.at<uchar>(10-1-i,j);
		}
	cout<<"temp="<<endl<<temp<<endl;

	
}



void xiu_RS_decoding(uchar* zeroones, uchar* output)  // 100 0s and 1s as input, output the string
{

	DmtxMessage* msg = dmtxMessageCreate(1,DmtxFormatMatrix);  // sizeIdx=1;


	memcpy(msg->array,zeroones, 100*sizeof(uchar));

	int sizeIdx=1; 
	int fix=-1;
	dmtxDecodePopulatedArray(sizeIdx, msg, fix); // Key

							printf("outputSize=%d \n", msg->outputSize); //output 1

							for(int i=0;i<(int)(msg->outputSize);i++)

							printf("fuck msg_xiu.output(%d)=%c \n", i, msg->output[i]);

	memcpy(output,msg->output, msg->outputSize*sizeof(uchar));
}


//=========================================================================================
//==========================================================================================  End of RS
//==========================================================================================




















//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//###############################################################      New TEST    function          ###############################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################

//##################################################################################################################################################################################################
//##################################################################################################################################################################################################

// new gausssian kernel
float New_GK[5][5]= {     99,     397,    695,    397,    99,
                	  397,   1986,   3277,   1986,   397,
                	  695,   3277,   5362,   3277,   695,
                	  397,   1986,   3277,   1986,   397,
                	  99,     397,    695,    397,    99};





//==================================================================================
//==================================================================================
// care the difference of  |x(i)-x(i-1)|

//uchar version
void New_Gaussian_Kernel_Blur_uchar(Mat& img, Mat& diff)
{	
	for(int i=2;i<img.rows-2;i++)
		for(int j=2;j<img.cols-2;j++)
		{
										

	diff.at<uchar>(i,j)=
      (uchar) ((  99* (float)img.at<uchar>(i-2,j-2) + 397*(float)img.at<uchar>(i-2,j-1) + 695*(float)img.at<uchar>(i-2,j) + 397*(float)img.at<uchar>(i-2,j+1) + 99*(float)img.at<uchar>(i-2,j+2)
		+ 397* (float)img.at<uchar>(i-1,j-2) + 1986*(float)img.at<uchar>(i-1,j-1) + 3277*(float)img.at<uchar>(i-1,j) + 1986*(float)img.at<uchar>(i-1,j+1) + 397*(float)img.at<uchar>(i-1,j+2) 
		+ 695* (float)img.at<uchar>(i,j-2) + 3277*(float)img.at<uchar>(i,j-1) + 5362*(float)img.at<uchar>(i,j) + 3277*(float)img.at<uchar>(i,j+1) + 695*(float)img.at<uchar>(i,j+2)  
		+ 397* (float)img.at<uchar>(i+1,j-2) + 1986*(float)img.at<uchar>(i+1,j-1) + 3277*(float)img.at<uchar>(i+1,j) + 1986*(float)img.at<uchar>(i+1,j+1) + 397*(float)img.at<uchar>(i+1,j+2)  
		+ 99* (float)img.at<uchar>(i+2,j-2) + 397*(float)img.at<uchar>(i+2,j-1) + 695*(float)img.at<uchar>(i+2,j) + 397*(float)img.at<uchar>(i+2,j+1) + 99*(float)img.at<uchar>(i+2,j+2))/pow(2,15));
		

		}		
									


}



//##################################################################################################################################################################################################
//##################################################################################################################################################################################################









//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//						     	READ .dat FILE
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################


#include<stdio.h>
#include<string.h>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>



						template<typename T>
						int ReadMatData_XIU(char* fileName, int rows, int cols, Mat& matData)
						{
	
							T* Buf=new uchar[rows*cols];



						//---------------------------read from file

							FILE *fp = NULL;  
							int read_length = 0;  
							fp = fopen(fileName, "r+");  
							if(fp == NULL)  
							{  
							    cout<<"file cannot be opened"<<endl;
								return 1;		 
							}  
							read_length = fread(Buf, 1, rows*cols, fp);  
							printf("read_length = %d\n", read_length);  
							fclose(fp);  
							fp = NULL;  

						//------------------------------------

							for (int r=0;r<rows;r++)
								{
									for(int c=0;c<cols;c++)
										{
											T data=Buf[r*matData.cols+c];
											matData.at<T>(r,c)=data;
										}
	
								}


							 
							return 1;
						}





//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//						   End   of READ .dat FILE
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################











//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//###############################################################      end of New TEST    function          ########################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################























//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################


//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################



//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################



//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int main(int argv, char** argc)
{

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ Part I   Read Image ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
							

								Mat img(480,640,CV_8UC1,Scalar(0));
		
								ReadMatData_XIU<uchar>("./wenshu/2Test1105.dat", 480,640, img); 
								
								imshow("img_read",img);
						
								imwrite("img.jpg",img);
	
								waitKey(0);
		
								

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ End of Part I   Read Image ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

				









				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ Part 2   Gaussian Blur added ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

					// replace this GB
					/*			Mat out;  
							    	GaussianBlur(img, out, Size(5, 5), 0, 0);
				
								img=out; // use the blur
								imshow("imgBlur",img);
								imwrite("imgBlur.jpg",img);
								waitKey(0);
					*/

					//replace this GB with DSP's GB output directly.
					
								Mat New_GK_out_uchar(img.size(),img.type(),Scalar(0));  
							    	New_Gaussian_Kernel_Blur_uchar(img, New_GK_out_uchar);
					/*
								img=New_GK_out_uchar;// assign img as blur out

								imshow("New_GK_out_uchar",New_GK_out_uchar);
								imwrite("New_GK_blur_uchar.jpg",New_GK_out_uchar);
								waitKey(0);
					*/
							
								
								Mat dspBlur(480,640,CV_8UC1,Scalar(0));
		
								ReadMatData_XIU<uchar>("./wenshu/GaussianBlur_2Test1105.dat", 480,640, dspBlur); 
								
								imshow("img_read_dspBlur",dspBlur);
		
								imwrite("dspBlur.jpg",dspBlur);

								img=dspBlur; // assign here
	
								waitKey(0);




								//compare the two gassian blur, print out i=100, j=100 to 200

								cout<<"New_GK_out_uchar.at(100,..)"<<endl;
								for(int j=100;j<120;j++)
									cout<<j<<"  "<<(int)New_GK_out_uchar.at<uchar>(100,j);

								cout<<endl<<endl<<"dspBlur.at(100,..)"<<endl;
								for(int j=100;j<120;j++)
									cout<<j<<"  "<<(int)dspBlur.at<uchar>(100,j);

		 						cout<<endl<<endl;
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ End of Part 2   Gaussian Blur added ^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 


				//>>>>>>>comment:  Blur is a Must.>>>>>>>>>>>>>>>>>>>>>>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%















							
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ Part 3  longitude subtraction ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

								Mat diff_long(img.size(),CV_32FC1,Scalar(0));
								longitudeScan(img,diff_long);

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ End of Part 3   longitude subtraction ^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%









				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ Part 4  vertical subtraction ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		
								Mat diff_ver(img.size(),CV_32FC1,Scalar(0));
								verticalScan(img,diff_ver);

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ End of Part 4   Gaussian Blur added ^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				















						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^^^^^^^^ Part 5  merge long and ver to get Gradient Value and Angle ^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

			
								Mat diff_long_ver(img.size(),CV_8UC1,Scalar(0));
								Mat diff_long_ver_float(img.size(),CV_32FC1,Scalar(0));
								Mat diff_long_ver_thr(img.size(),CV_8UC1,Scalar(0));
								Mat Angle(img.size(),CV_32FC1,Scalar(0)); // -180 ~ 180 for here acting on abs, so only 0~180

								for(int i=0;i<diff_long_ver.rows;i++)
									for(int j=0;j<diff_long_ver.cols;j++)
										{
											diff_long_ver.at<uchar>(i,j)= (uchar)sqrt ( (diff_long.at<float>(i,j) *diff_long.at<float>(i,j)
															 +  diff_ver.at<float>(i,j) *diff_ver.at<float>(i,j) ) );  //Tested Correct

											diff_long_ver_float.at<float>(i,j)= (float)sqrt ( (diff_long.at<float>(i,j) *diff_long.at<float>(i,j)
															 +  diff_ver.at<float>(i,j) *diff_ver.at<float>(i,j) ) );  //Tested Correct 

											Angle.at<float>(i,j)=(180/PI)*atan2( (float) diff_ver.at<float>(i,j), (float)diff_long.at<float >(i,j) ); 
																						   //Tested Correct 

											if(diff_long_ver.at<uchar>(i,j)<20)
												diff_long_ver_thr.at<uchar>(i,j)=0;
											else
												diff_long_ver_thr.at<uchar>(i,j)=255; // Tested Correct
												
																					
													
										}
	


								

								imshow("thr",diff_long_ver_thr);
								imwrite("thr.jpg",diff_long_ver_thr);
								imwrite("diff_long_ver.jpg",diff_long_ver);
								waitKey(0);


						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^ End of Part 5  merge long and ver to get Gradient Value and Angle ^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	










/*


						
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^^^^^^^^ Part 6  Given a StartPoint, Find Next and Next             ^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

						// global var in finding
						
						float theta_updated=0;

						Mat diff_long_ver_copy_1= diff_long_ver.clone();  // preserve diff_long_ver

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&			Test The find next loop        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   

						// loop find the Next

							Point P(86,132);    //P(94,140) for st_raw
							Point P_Next(0,0);

							for(int i=0;i<3;i++)
							{								
								theta_updated= Angle.at<float>(P.y,P.x);       // initialize theta_updated
		
								FindNextPoint(P, P_Next,  diff_long_ver_float,  theta_updated);


										//=========draw P_Next out
									 	circle(diff_long_ver_copy_1, P_Next, 5, Scalar(255), 1, 8, 0);
				
										cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
										    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
										cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

										imshow("diff_long_ver",diff_long_ver);
										imwrite("diff_long_ver.jpg",diff_long_ver);
										waitKey(0);
										//==========end of draw P out

								P=P_Next;  // iterate
					
							}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	end of Test The find next loop        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   //Test Corrected






						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^ End of Part 6  Given a StartPoint, Find Next and Next 	 ^^^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^











//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%











						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^^^   Part 7  Given a StartPoint, Jugde whether it is on a Line.    ^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Test The find next Right, Left        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   


						Mat diff_long_ver_copy_2= diff_long_ver_copy_1.clone();

						P=Point(159,131);    //P(94,140) for st_raw
						P_Next=Point(0,0);
						
						
						for(int i=0;i<10;i++)
							{								
								theta_updated= Angle.at<float>(P.y,P.x);       // initialize theta_updated
		
								FindNextPoint_Left_Big(P, P_Next,  diff_long_ver_float,  theta_updated);


										//=========draw P_Next out on 
									 	circle(diff_long_ver_copy_2, P_Next, 3, Scalar(255), 1, 8, 0);
				
										cout<<"row,col, angle=  Value= "<<P_Next.y<<"  "<<P_Next.x<<"  "<<Angle.at<float>(P_Next.y,P_Next.x)
										    <<"  "<<(float)diff_long_ver_float.at<float>(P_Next.y,P_Next.x)<<endl;
										cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl<<endl;

										imshow("diff_long_ver_copy_2",diff_long_ver_copy_2);
										imwrite("diff_long_ver_copy_2.jpg",diff_long_ver_copy_2);
										waitKey(50);
										//==========end of draw P out

								P=P_Next;  // iterate
					
							}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  end of Test The find next loop        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   //Test Corrected

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------








//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Test JudgeSeed_On_A_Line        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  


						Point Seed=Point(159,131);
	
						JudgeSeed_On_A_Line (Seed,  LineSeg& line_seg, diff_long_ver_float, Angle, diff_long_ver);

/*						//--------
				
						Point Seed_1= Point(94,140);

						JudgeSeed_On_A_Line (Seed_1, diff_long_ver_float, Angle, diff_long_ver);
		

						Point Seed_2= Point(158,144);

						JudgeSeed_On_A_Line (Seed_2, diff_long_ver_float, Angle, diff_long_ver);


						Point Seed_3= Point(432,394);

						JudgeSeed_On_A_Line (Seed_3, diff_long_ver_float, Angle, diff_long_ver);



						
						Point Seed_4= Point(192,101);

						JudgeSeed_On_A_Line (Seed_4, diff_long_ver_float, Angle, diff_long_ver);


						Point Seed_5= Point(296,132);

						JudgeSeed_On_A_Line (Seed_5, diff_long_ver_float, Angle, diff_long_ver);

*/
// tested Correct!    All perfect!
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  end of JudgeSeed_On_A_Line        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  


						



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



/*


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Test TwoEndFinding        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  


						Point Seed_forEnd=Point(159,131);
			
						Point End_Left(0,0), End_Right(0,0);

						TwoEndFinding(Seed_forEnd, End_Left, End_Right,     diff_long_ver_float, Angle, diff_long_ver );
							
						// output: End_Left: 73,132  optimized to the following

						//End_Left=[69, 134]        End_Right=[201, 131]





// tested correct
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  end of TwoEndFinding        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


*/




//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------








//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Test L        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  

						Point Seed=Point(372,297);

						Ten_Rits  Ten_Rits_Gold_OutPut={ Point2f(0,0),Point2f(0,0),Point2f(0,0),Point2f(0,0),Point2f(0,0)};

						L( Seed,  Ten_Rits_Gold_OutPut,       diff_long_ver_float, Angle,    diff_long_ver);

							


// ---------------------------------------------Up to Now, from only one Seed, we get the Ten Rits -----------------------------------------------------------------------
			


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  end of Test L        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&






//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------












//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Draw Color      &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  



						Mat bina= imread("2cm1017_bina.jpg",0);

							// big bug , here bina need clean to pure 0 and pure 255;

							//clean

								for(int i=0;i<bina.rows;i++)

									for(int j=0;j<bina.cols;j++)
									{
										if(bina.at<uchar>(i,j) >200)	
											bina.at<uchar>(i,j)=255;
										else
											bina.at<uchar>(i,j)=0;
									}
		
							//end of clean bina


						Mat img_colorline(img.size(), CV_8UC3, Scalar(0,0,0));

						Mat diff_long_ver_copy_2= diff_long_ver.clone();
	

						for(int i=0;i<img_colorline.rows;i++)

							for(int j=0;j<img_colorline.cols;j++)
							{
								Point temp=Point(j,i);
								img_colorline.at<Vec3b>(temp.y, temp.x)[0]=bina.at<uchar>(temp.y, temp.x);
								img_colorline.at<Vec3b>(temp.y, temp.x)[1]=bina.at<uchar>(temp.y, temp.x);
								img_colorline.at<Vec3b>(temp.y, temp.x)[2]=bina.at<uchar>(temp.y, temp.x);

							}




						for(int i=0;i<10;i++)

							for(int j=0;j<10;j++)

							{
								Point2f corner= Ten_Rits_Gold_OutPut.Origin  + (1.5+i)* Ten_Rits_Gold_OutPut.StepVec1 + (1.5+j)* Ten_Rits_Gold_OutPut.StepVec2	;

								Point P_Draw (cvRound(corner.x), cvRound(corner.y));		

								Point P_Next= P_Draw;

						
				

										//=========draw P_Next out on 
										
										
		
									 	circle(img_colorline, P_Next, 1, Scalar(i*20,255,0), 1, 8, 0);
									

										imshow("img_colorline",img_colorline);
										imwrite("bina_drawP_color.jpg",img_colorline);
										waitKey(50);
										//==========end of draw P out
							}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& end of Draw Color      &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  































//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//                                                                     RS From Here
		

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Lift 0 and 1     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  


//####################  OK, Let's Find the bits 0s and 1s #############################################################

						cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  begin to calculate all the 0s and 1s -------"<<endl;


						Mat bina_final=bina.clone();

						Mat decodeResult= (Mat_<uchar>(10,10)<<0);

						Point2f unit_first= Ten_Rits_Gold_OutPut.StepVec1;
						Point2f unit_second= Ten_Rits_Gold_OutPut.StepVec2;

						Point2f codebasep=Ten_Rits_Gold_OutPut.Origin+ 1.5*unit_second + 1.5*unit_first;  //drift add here, above is 2.5, here for code is 3.5

						for(int fi=0;fi<10;fi++)
							for(int fj=0;fj<10;fj++)
	
							{
								Point2f temp= codebasep + fi*unit_first +fj*unit_second;
	
								if(bina_final.at<uchar>( cvRound(temp.y) , cvRound(temp.x) ) ==255 )
									decodeResult.at<uchar>(fi,fj)=1;
								else
									decodeResult.at<uchar>(fi,fj)=0;	
		
		
							}


						cout<<"decodeResult="<<endl<<decodeResult<<endl;




						cout<<"==========================fuck out the final fuck up 0s and 1s ============================================"<<endl;







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;

						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;

						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
						cout<<"==========================Reed Solomn Decoding here============================================"<<endl;


						uchar zeroones_after_formsg[100]={0};


						reorder( decodeResult, zeroones_after_formsg);

						uchar output[120]={0};

						xiu_RS_decoding( zeroones_after_formsg, output);










						cout<<"==========================final decoded story is============================================"<<endl;
						cout<<"==========================final decoded story is============================================"<<endl;
						cout<<"==========================final decoded story is============================================"<<endl;
						cout<<"==========================final decoded story is============================================"<<endl;


						for(int ifi=0;ifi<120;ifi++)
							if(output[ifi]!=0)
								cout<<output[ifi];
						cout<<endl;
						cout<<endl;
						cout<<endl;

						cout<<"============================================================================================="<<endl;







//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//                                                                     RS End Here
		

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%









	return 1;
} 











//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################




























