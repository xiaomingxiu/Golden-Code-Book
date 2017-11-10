// sobel

#define HAVE_OPENCV_FLANN
#define HAVE_OPENCV_IMGPROC
#define HAVE_OPENCV_VIDEO
#define HAVE_OPENCV_OBJDETECT
#define HAVE_OPENCV_CALIB3D
#define HAVE_OPENCV_ML
#define HAVE_OPENCV_HIGHGUI
#define HAVE_OPENCV_CONTRIB

#include<math.h>
#include<opencv2/opencv.hpp>

#include<iostream>

#include<vector>      

using namespace std;
using namespace cv;

const float PI=3.14159265;
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

							float stepLen=2; // pixel ; 10.29  chang from 3 to 2;

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
						int GetFindNum_Right_Big ( Point Seed, Point& FinalEnd_Right_Big, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfRight_Big=0;

							//
							float theta_updated=Angle.at<float>(Seed.y,Seed.x);       //initialize theta_updated; 

							float theta_last=0;

							Mat diff_long_ver_copy=diff_long_ver.clone();

							//

							Point P=Seed;    
							Point P_Next(0,0);

							for(int i=0;i<20;i++)
							{
								FindNextPoint_Right_Big(P, P_Next,  diff_long_ver_float,  theta_updated);

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
	



										numOfRight_Big++;
										
										P=P_Next;  // iterate									
						
								}		

						
								else

									break;
					
							}

							// final end point					
							FinalEnd_Right_Big = P;

							return numOfRight_Big;

							

						}   
			
						// end of GetFindNum_Right_Big()
						//----------------------------------------------------------






						// IMPORTANT: Core  :---- Get Num   Right_Big     //also output a FinalEnd_Right Point
						int GetFindNum_Right_Small ( Point Seed, Point& FinalEnd_Right_Small, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
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
						int GetFindNum_Left_Big ( Point Seed, Point& FinalEnd_Left_Big, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							int numOfLeft_Big=0;

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
										
										P=P_Next;  // iterate									
						
								}		

						
								else

									break;
					
							}

							// final end point
							FinalEnd_Left_Big= P;

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











						bool  JudgeSeed_On_A_Line(Point Seed, Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver )
															    // here diff_long_ver only for draw, will omitted				
						{

							// continuous successful trace steps nums as criterion
							int numOfRight_Big=0, numOfRight_Small=0;
							int numOfLeft_Big=0, numOfLeft_Small=0;
							int numOfBig=0, numOfSmall=0;

							Point FinalEnd_Left_Big=Point(0,0);
							Point FinalEnd_Right_Big=Point(0,0);
							
							// Get Num :
							// Num_Right_Big  and Final end
							
							numOfRight_Big = GetFindNum_Right_Big ( Seed, FinalEnd_Right_Big,diff_long_ver_float, Angle,  diff_long_ver); 

								cout<<endl<<endl<<"^^^^^^^^^^^^^^"<<endl;
								cout<<"numOfRight_Big for Point "<<Seed<< " = "<<numOfRight_Big<<endl;
								cout<<"FinalEnd_Right_Big="<<FinalEnd_Right_Big<<endl;
								cout<<"^^^^^^^^^^^^^^^^"<<endl<<endl;

							// Num_Right_Big  and Final end
							numOfLeft_Big = GetFindNum_Left_Big ( Seed, FinalEnd_Left_Big, diff_long_ver_float, Angle,  diff_long_ver); 

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
									return true;
								}	

							else
								{
									cout<<endl<<endl<<" -----------------------------"<<endl;
									cout<<"Oh NO, it is  not on A Line !     with numOfBig="<< numOfBig<<endl<<endl;
									return false;
								}
							
							
		
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
						Point LeftEndFinding (Point Seed,      Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							// first get Left_Big jump end;
							Point FinalEnd_Left_Big=Point(0,0);
							GetFindNum_Left_Big ( Seed, FinalEnd_Left_Big, diff_long_ver_float, Angle, diff_long_ver);


							//second , small jump to confine
							Point Seed_After_BigJump = FinalEnd_Left_Big;  // initiallise for small jump
							
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
						Point RightEndFinding (Point Seed,      Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver)
						{
							// first get Right_Big jump end;
							Point FinalEnd_Right_Big=Point(0,0);
							GetFindNum_Right_Big ( Seed, FinalEnd_Right_Big, diff_long_ver_float, Angle, diff_long_ver);


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




						//input: one seed
						//output: two ends

						// precondition: it is on a line

						void TwoEndFinding(Point Seed, Point& End_Left, Point& End_Right,     Mat& diff_long_ver_float, Mat& Angle, Mat& diff_long_ver )
	
						{
							
							End_Left= LeftEndFinding(Seed,      diff_long_ver_float, Angle, diff_long_ver);
	
											
							End_Right= RightEndFinding(Seed,    diff_long_ver_float, Angle, diff_long_ver);
							
							cout<<"-------------"<<endl;	
							cout<<"End_Left="<<End_Left<<endl;

							cout<<"-------------"<<endl;	
							cout<<"End_Right="<<End_Right<<endl;
						
						}


						
						


		





//##############################################           end   of    core : TwoEndFinding()	########################
//##############################################################################################################################










//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   End of Core Function   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
















//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################









//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int main()
{

				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^ Part I   Read Image ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
								Mat img=imread("frame0.jpg",0);
	
	
								imshow("img",img);
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

								Mat out;  
							    	GaussianBlur(img, out, Size(5, 5), 0, 0);
				
								img=out; // use the blur
								imshow("imgBlur",img);
								imwrite("imgBlur.jpg",img);
								waitKey(0);
		 
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
								waitKey(0);


						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
						//^^^^^^^^^^^^^^ End of Part 5  merge long and ver to get Gradient Value and Angle ^^^
						//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	













						
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

/*
						Point Seed=Point(159,131);
	
						JudgeSeed_On_A_Line (Seed, diff_long_ver_float, Angle, diff_long_ver);

						//--------
				
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






//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  Test TwoEndFinding        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  


						Point Seed_forEnd=Point(159,131);
			
						Point End_Left(0,0), End_Right(0,0);

						TwoEndFinding(Seed_forEnd, End_Left, End_Right,     diff_long_ver_float, Angle, diff_long_ver );
							
						// output: End_Left: 73, 132

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	  end of TwoEndFinding        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



	return 1;
} 











//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################
//##################################################################################################################################################################################################




























