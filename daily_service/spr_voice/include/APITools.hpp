#include<opencv2/opencv.hpp>  
#include<iostream>  
#include<vector>  
#include<string>  
#include<curl/curl.h>
#include<Zbase64.hpp>
#include <jsoncpp/json/json.h>
using namespace std;
using namespace cv;
int callback_m(char *data, size_t size, size_t nmemb, string *onyxiaData){
	if (onyxiaData == NULL) return 0;
	int len = size * nmemb;
	onyxiaData->append(data, len);
	return len;
}

class faceplusplusApi{
public:
	faceplusplusApi();
	~faceplusplusApi();

	float faceCompare(Mat inputimg, Mat compareimg);
	void facedetect(Mat inputimg, int num[5]);
	void face_label(Mat inputimg);
    int humanProp(Mat inputimg);
    int skeletondetect(Mat inputimg, int (&arr)[20]);
private:
};
faceplusplusApi::faceplusplusApi()
{
}
faceplusplusApi::~faceplusplusApi()
{
}


inline float faceplusplusApi::faceCompare(Mat inputimg, Mat compareimg)
{
	string buffer;
	CURL *curl;
	CURLcode res;
	struct curl_slist *http_header = NULL;
	struct curl_httppost *formpost = 0;
	struct curl_httppost *lastptr = 0;
	curl = curl_easy_init();
	vector<uchar> vecSrc, vecComp;                               //Mat 图片数据转换为vector<uchar>  
	vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imencode(".jpg", inputimg, vecSrc, vecCompression_params);
	imencode(".jpg", compareimg, vecComp, vecCompression_params);

	ZBase64 base64;
	String Src_base64 = base64.Encode(vecSrc.data(), vecSrc.size());     //实现图片的base64编码  
	String Comp_base64 = base64.Encode(vecComp.data(), vecComp.size());
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_key",
		CURLFORM_COPYCONTENTS, "-ka6VnW4m7QL0-RZxKs7s7nfNdH0Wq_h",//"uBBpU3fFfl7VBXDqj4hPhtNXDqF8z6pk",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_secret",
		CURLFORM_COPYCONTENTS, "6jPftl6ERGPd14PIfkWslMtbCe6wdP28",//"r4JNa3CFTdScenHQ7ks544Y9LURbVWhm",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64_1",
		CURLFORM_COPYCONTENTS, Src_base64.c_str(),
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64_2",
		CURLFORM_COPYCONTENTS, Comp_base64.c_str(),
		CURLFORM_END);

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_m);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
	curl_easy_setopt(curl, CURLOPT_URL, "https://api-cn.faceplusplus.com/facepp/v3/compare");
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
	res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);
	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(buffer, root);
	if (!parsingSuccessful){
		std::cout << "Failed to parse the data!" << std::endl;
		exit(0);
	}

	float out = root["confidence"].asInt();
	return out;
}


inline void faceplusplusApi::facedetect(Mat inputimg, int num[5])
{

	vector<uchar> vecImg;                               //Mat 图片数据转换为vector<uchar>  
	vector<int> vecCompression_params;
	vecCompression_params.push_back(95);
	vecCompression_params.push_back(90);
	imencode(".jpeg", inputimg, vecImg, vecCompression_params);

	ZBase64 base64;
	String imgbase64 = base64.Encode(vecImg.data(), vecImg.size());     //实现图片的base64编码  
	string buffer;
	

	CURL *curl;
	CURLcode res;
	struct curl_slist *http_header = NULL;
	struct curl_httppost *formpost = 0;
	struct curl_httppost *lastptr = 0;
	curl = curl_easy_init();
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_key",
		CURLFORM_COPYCONTENTS, "-ka6VnW4m7QL0-RZxKs7s7nfNdH0Wq_h",//"uBBpU3fFfl7VBXDqj4hPhtNXDqF8z6pk",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_secret",
		CURLFORM_COPYCONTENTS, "6jPftl6ERGPd14PIfkWslMtbCe6wdP28",//"r4JNa3CFTdScenHQ7ks544Y9LURbVWhm",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64",
		CURLFORM_COPYCONTENTS, imgbase64.c_str(),
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "return_attributes",
		CURLFORM_COPYCONTENTS, "gender",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "calculate_all",//仅可正式API使用，试用请注释
		CURLFORM_COPYCONTENTS, "1",
		CURLFORM_END);

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_m);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
	curl_easy_setopt(curl, CURLOPT_URL, "https://api-cn.faceplusplus.com/facepp/v3/detect");
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
	res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);

	Json::Value detect;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(buffer, detect);

	if (!parsingSuccessful){
		std::cout << "Failed to parse the data!" << std::endl;
		exit(0);
	}
        //cout << buffer << endl;

	int face_size = detect["faces"].size();
	//cv::Rect face_rect[face_size];   

	printf("共检测到%d张人脸\n", face_size);
	//printf("性别分别为：\n");
	int male=0,female=0;
	string gender;
    int standcount=0, sitcount=0;
	for (int i = 0; i < face_size; i++)
	{
		Json::Value attribute;
		attribute = detect["faces"][i]["attributes"];
		gender = attribute["gender"]["value"].asString();
		//genders = detect["faces"][i]["gender"]["value"].asString();
		cout << gender << endl;
		int x, y, width, height;
		
		x = detect["faces"][i]["face_rectangle"]["left"].asInt();
		y = detect["faces"][i]["face_rectangle"]["top"].asInt();
		width = detect["faces"][i]["face_rectangle"]["width"].asInt();
		height = detect["faces"][i]["face_rectangle"]["height"].asInt();
		if ((x > 1) && (x + width < inputimg.cols) && (y > 1) && (y + height < inputimg.rows))
		{
			cv::rectangle(inputimg, Rect(x, y, width, height), cv::Scalar(0, 240, 0), 2);
		//	cv::Mat roi_img = inputimg(face_rect[i]);
			cv::putText(inputimg, gender, Point(x + 15, y + 15), CV_FONT_NORMAL, 0.6, Scalar(0, 0, 240)); 
		}
		
		if(y > inputimg.rows*0.3) sitcount++; else standcount++; 
		string sex="Male";
		if (sex.compare(gender) == 0) {male++;}
		else {female++;}
	}
	//printf("男性人数为：%d, 女性人数为：%d",male,female);
	//printf("\n");
	num[0]= face_size;
	num[1]= male;
	num[2]= female;
	num[3]= standcount;
	num[4]= sitcount;
}






inline void faceplusplusApi::face_label(Mat inputimg)
{
	vector<uchar> vecImg;                               //Mat 图片数据转换为vector<uchar>  
	vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imencode(".jpg", inputimg, vecImg, vecCompression_params);

	ZBase64 base64;
	String imgbase64 = base64.Encode(vecImg.data(), vecImg.size());     //实现图片的base64编码  
	string buffer;
	CURL *curl;
	CURLcode res;
	struct curl_slist *http_header = NULL;
	struct curl_httppost *formpost = 0;
	struct curl_httppost *lastptr = 0;
	curl = curl_easy_init();
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_key",
		CURLFORM_COPYCONTENTS, "uBBpU3fFfl7VBXDqj4hPhtNXDqF8z6pk",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_secret",
		CURLFORM_COPYCONTENTS, "r4JNa3CFTdScenHQ7ks544Y9LURbVWhm",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64",
		CURLFORM_COPYCONTENTS, imgbase64.c_str(),
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "return_attributes",
		CURLFORM_COPYCONTENTS, "gender",
		CURLFORM_END);

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_m);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
	curl_easy_setopt(curl, CURLOPT_URL, "https://api-cn.faceplusplus.com/facepp/v3/detect");
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
	res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);

	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(buffer, root);

	if (!parsingSuccessful){
		std::cout << "Failed to parse the data!" << std::endl;
		exit(0);
	}

	cv::Rect face_rect[1]; 
	string faces_path = NULL;
	cv::Mat faces_img=cv::imread(faces_path);
	string name_str = NULL;
	vector<int> pixel_all;
	for (int i = 0; i < 10; i++) {
		face_rect[i].x = root["faces"][i]["face_rectangle"]["left"].asInt();
		face_rect[i].y = root["faces"][i]["face_rectangle"]["top"].asInt();
		face_rect[i].width = root["faces"][i]["face_rectangle"]["width"].asInt();
		face_rect[i].height = root["faces"][i]["face_rectangle"]["height"].asInt();	

		if ((face_rect[i].x>1)&&(face_rect[i].x+ face_rect[i].width<inputimg.cols)&&(face_rect[i].y>1)&&(face_rect[i].y+ face_rect[i].height<inputimg.rows)){
			cv::rectangle(inputimg, face_rect[i], Scalar(0, 240, 0), 2);

			cv::Mat roi_img = inputimg(face_rect[i]);
			if(faceCompare(roi_img,faces_img)>65){ 
				cv::putText(inputimg, name_str, Point(face_rect[i].x + 25, face_rect[i].y + 25), CV_FONT_NORMAL, 0.6, Scalar(0, 0, 240)); 	
			}

			cv::imwrite("ZZU-SR_Try-face.jpg", inputimg);
		}	
	}


}

inline int faceplusplusApi::humanProp(Mat inputimg)
{
	vector<uchar> vecImg;                                 
	vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imencode(".jpg", inputimg, vecImg, vecCompression_params);

	ZBase64 base64;
	String imgbase64 = base64.Encode(vecImg.data(), vecImg.size());       
	string buffer;
	CURL *curl;
	CURLcode res;
	struct curl_slist *http_header = NULL;
	struct curl_httppost *formpost = 0;
	struct curl_httppost *lastptr = 0;
	curl = curl_easy_init();
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_key",
		CURLFORM_COPYCONTENTS, "-ka6VnW4m7QL0-RZxKs7s7nfNdH0Wq_h",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_secret",
		CURLFORM_COPYCONTENTS, "6jPftl6ERGPd14PIfkWslMtbCe6wdP28",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64",
		CURLFORM_COPYCONTENTS, imgbase64.c_str(),
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "return_attributes",
		CURLFORM_COPYCONTENTS, "gender","upper_body_cloth","lower_body_cloth",
		CURLFORM_END);

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_m);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
	curl_easy_setopt(curl, CURLOPT_URL, "https://api-cn.faceplusplus.com/humanbodypp/v1/detect");
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
	res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);

	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(buffer, root);

	if (!parsingSuccessful)
	{
		std::cout << "Failed to parse the data!" << std::endl;
		exit(0);
	}
	
	for (int i = 0; i < root["humanbodies"].size(); i++){
		string gender = root["humanbodies"][i]["attributes"]["gender"].asString();//["value"]
		string upperColor =  root["humanbodies"][i]["attributes"]["upper_body_cloth"]["upper_body_cloth_color"].asString();
		string lowerColor =  root["humanbodies"][i]["attributes"]["lower_body_cloth"]["lower_body_cloth_color"].asString();
		string info = gender +" "+upperColor+ " "+lowerColor;
		cout<< info <<endl;
		
		int x, y, width, height;
		x = root["humanbodies"][i]["humanbody_rectangle"]["left"].asInt();
		y = root["humanbodies"][i]["humanbody_rectangle"]["top"].asInt();
		width = root["humanbodies"][i]["humanbody_rectangle"]["width"].asInt();
		height = root["humanbodies"][i]["humanbody_rectangle"]["height"].asInt();
		if ((x > 1) && (x + width < inputimg.cols) && (y > 1) && (y + height < inputimg.rows))
		{
			cv::rectangle(inputimg, Rect(x, y, width, height), cv::Scalar(0, 240, 0), 2);
			cv::putText(inputimg, info, Point(x + 15, y + 15), CV_FONT_NORMAL, 0.4, Scalar(0, 0, 240)); 
		}
	}
	
	return 0;

}

inline int faceplusplusApi::skeletondetect(Mat inputimg, int (&arr)[20]){
	
	vector<uchar> vecImg;                               
	vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imencode(".jpg", inputimg, vecImg, vecCompression_params);

	ZBase64 base64;
	String imgbase64 = base64.Encode(vecImg.data(), vecImg.size());     
	string buffer;
	CURL *curl;
	CURLcode res;
	struct curl_slist *http_header = NULL;
	struct curl_httppost *formpost = 0;
	struct curl_httppost *lastptr = 0;
	curl = curl_easy_init();
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_key",
		CURLFORM_COPYCONTENTS, "-ka6VnW4m7QL0-RZxKs7s7nfNdH0Wq_h",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "api_secret",
		CURLFORM_COPYCONTENTS, "6jPftl6ERGPd14PIfkWslMtbCe6wdP28",
		CURLFORM_END);
	curl_formadd(&formpost,
		&lastptr,
		CURLFORM_COPYNAME, "image_base64",
		CURLFORM_COPYCONTENTS, imgbase64.c_str(),
		CURLFORM_END);

	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_m);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
	curl_easy_setopt(curl, CURLOPT_URL, "https://api-cn.faceplusplus.com/humanbodypp/v1/skeleton");
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
	res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);

	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(buffer, root);

	if (!parsingSuccessful)
	{
		std::cout << "Failed to parse the data!" << std::endl;
		exit(0);
	}
    int person_size = root["skeletons"].size();
	
			
	for (int i = 0; i < person_size; i++){
		int x, y, height, width;
		x = root["skeletons"][i]["body_rectangle"]["left"].asInt();
		y = root["skeletons"][i]["body_rectangle"]["top"].asInt();
		height = root["skeletons"][i]["body_rectangle"]["height"].asInt();
		width = root["skeletons"][i]["body_rectangle"]["width"].asInt();
		
		int head_x = root["skeletons"][i]["landmark"]["head"]["x"].asInt();
		int head_y = root["skeletons"][i]["landmark"]["head"]["y"].asInt();
		int neck_x = root["skeletons"][i]["landmark"]["neck"]["x"].asInt();
		int neck_y = root["skeletons"][i]["landmark"]["neck"]["y"].asInt();
		 int left_shoulder_x = root["skeletons"][i]["landmark"]["left_shoulder"]["x"].asInt();
		 int left_shoulder_y = root["skeletons"][i]["landmark"]["left_shoulder"]["y"].asInt();
		int right_shoulder_x = root["skeletons"][i]["landmark"]["right_shoulder"]["x"].asInt();
		int right_shoulder_y = root["skeletons"][i]["landmark"]["right_shoulder"]["y"].asInt();
		 int left_elbow_x = root["skeletons"][i]["landmark"]["left_elbow"]["x"].asInt();
		 int left_elbow_y = root["skeletons"][i]["landmark"]["left_elbow"]["y"].asInt();
		int right_elbow_x = root["skeletons"][i]["landmark"]["right_elbow"]["x"].asInt();
		int right_elbow_y= root["skeletons"][i]["landmark"]["right_elbow"]["y"].asInt();
		 int left_hand_x = root["skeletons"][i]["landmark"]["left_hand"]["x"].asInt();
		 int left_hand_y = root["skeletons"][i]["landmark"]["left_hand"]["y"].asInt();
		int right_hand_x = root["skeletons"][i]["landmark"]["right_hand"]["x"].asInt();
		int right_hand_y = root["skeletons"][i]["landmark"]["right_hand"]["y"].asInt();
		 int left_buttocks_x = root["skeletons"][i]["landmark"]["left_buttocks"]["x"].asInt();
		 int left_buttocks_y = root["skeletons"][i]["landmark"]["left_buttocks"]["y"].asInt();
		int right_buttocks_x = root["skeletons"][i]["landmark"]["right_buttocks"]["x"].asInt();
		int right_buttocks_y = root["skeletons"][i]["landmark"]["right_buttocks"]["y"].asInt();
		 int left_knee_x = root["skeletons"][i]["landmark"]["left_knee"]["x"].asInt();
		 int left_knee_y = root["skeletons"][i]["landmark"]["left_knee"]["y"].asInt();
		int right_knee_x = root["skeletons"][i]["landmark"]["right_knee"]["x"].asInt();
		int right_knee_y = root["skeletons"][i]["landmark"]["right_knee"]["y"].asInt(); 
		 int left_foot_x = root["skeletons"][i]["landmark"]["left_foot"]["x"].asInt();
		 int left_foot_y = root["skeletons"][i]["landmark"]["left_foot"]["y"].asInt();
		int right_foot_x = root["skeletons"][i]["landmark"]["right_foot"]["x"].asInt();
		int right_foot_y = root["skeletons"][i]["landmark"]["right_foot"]["y"].asInt();
		
		double left_heightf = (double) (left_knee_y - head_y);
		double right_heightf = (double) (right_knee_y - head_y);
		
		cout << "left_height : " << left_heightf << endl;
		cout << "right_height : " << right_heightf << endl;
		cout << "head_x : " << x+head_x << ", " << "head_y : " << y+head_y << endl;
		cout << "neck_x : " << x+neck_x << ", " << "neck_y : " << y+neck_y << endl;
		cout << "left_shoulder_x : " << x+left_shoulder_x << ", " << "left_shoulder_y : " << y+left_shoulder_y << endl;
		cout << "right_shoulder_x : " << x+right_shoulder_x << ", " << "right_shoulder_y : " << y+right_shoulder_y << endl;
		cout << "left_elbow_x : " << x+left_elbow_x << ", " << "left_elbow_y : " << y+left_elbow_y << endl;
		cout << "right_elbow_x : " << x+right_elbow_x << ", " << "right_elbow_y : " << y+right_elbow_y << endl;
		cout << "left_hand_x : " << x+left_hand_x << ", " << "left_hand_y : " << y+left_hand_y << endl;
		cout << "right_hand_x : " << x+right_hand_x << ", " << "right_hand_y : " << y+right_hand_y << endl;
		cout << "left_buttocks_x : " << x+left_buttocks_x << ", " << "left_buttocks_y : " << y+left_buttocks_y << endl;
		cout << "right_buttocks_x : " << x+right_buttocks_x << ", " << "right_buttocks_y : " << y+right_buttocks_y << endl;
		cout << "left_knee_x : " << x+left_knee_x << ", " << "left_knee_y : " << y+left_knee_y << endl;
		cout << "right_knee_x : " << x+right_knee_x << ", " << "right_knee_y : " << y+right_knee_y << endl;
		cout << "left_foot_x : " << x+left_foot_x << ", " << "left_foot_y : " << y+left_foot_y << endl;
		cout << "right_foot_x : " << x+right_foot_x << ", " << "right_foot_y : " << y+right_foot_y << endl;
		cout << "The quotient_y between buttocks and knee : " << "left : " << (left_knee_y - left_buttocks_y) / left_heightf << ", right : " << (right_knee_y - right_buttocks_y) / right_heightf << endl;
		
		
		//opencv点标注
		Point phead = Point (x + head_x, y + head_y);
		Point pneck = Point (x + neck_x, y + neck_y);
		Point pleft_shoulder = Point (x + left_shoulder_x, y + left_shoulder_y);
		Point pright_shoulder = Point (x + right_shoulder_x, y + right_shoulder_y);
		Point pleft_elbow = Point (x + left_elbow_x, y + left_elbow_y);
		Point pright_elbow = Point (x + right_elbow_x, y + right_elbow_y);
		Point pleft_hand = Point (x + left_hand_x, y + left_hand_y);
		Point pright_hand = Point (x + right_hand_x, y + right_hand_y);
		Point pleft_buttocks = Point (x + left_buttocks_x, y + left_buttocks_y);
		Point pright_buttocks = Point (x + right_buttocks_x, y + right_buttocks_y);
		Point pleft_knee = Point (x + left_knee_x, y + left_knee_y);
		Point pright_knee = Point (x + right_knee_x, y + right_knee_y);
		Point pleft_foot = Point (x + left_foot_x, y + left_foot_y);
		Point pright_foot = Point (x + right_foot_x, y + right_foot_y);
		
		
		if ((x > 1) && (x + width < inputimg.cols) && (y > 1) && (y + height < inputimg.rows)){
			cv::rectangle(inputimg, Rect(x, y, width, height+15), cv::Scalar(0, 240, 0), 2);
			
			string person_id = "Person: "+ to_string(i);
			putText(inputimg, person_id, Point(x - 30, y + 20), CV_FONT_HERSHEY_COMPLEX, 0.65, Scalar(0, 0, 255), 2); 
			
			
			
			
			//~ if(((left_knee_y - left_buttocks_y)<50) && ((right_knee_y - right_buttocks_y)<50)){
				//~ putText(inputimg, "sit", Point(x + 15, y + 15), CV_FONT_HERSHEY_COMPLEX, 1.5, Scalar(0, 0, 255), 2); 
			//~ )
			
			
			
			
			//opencv直线标注
			line(inputimg, phead, pneck, Scalar(255,191,0),4);
			line(inputimg, pneck, pleft_shoulder, Scalar(0,255,255),4);
			line(inputimg, pneck, pright_shoulder, Scalar(0,255,0),4);
			line(inputimg, pleft_hand, pleft_elbow, Scalar(255,191,0),4);
			line(inputimg, pleft_elbow, pleft_shoulder, Scalar(0,255,255),4);
			line(inputimg, pright_hand, pright_elbow, Scalar(0,255,0),4);
			line(inputimg, pright_elbow, pright_shoulder, Scalar(255,191,0),4);
			line(inputimg, pleft_buttocks, pleft_shoulder, Scalar(0,255,255),4);
			line(inputimg, pright_buttocks, pright_shoulder, Scalar(0,255,0),4);
			line(inputimg, pleft_foot, pleft_knee, Scalar(255,191,0),4);
			line(inputimg, pleft_buttocks, pleft_knee, Scalar(0,255,255),4);
			line(inputimg, pright_foot, pright_knee, Scalar(0,255,0),4);
			line(inputimg, pright_buttocks, pright_knee, Scalar(255,191,0),4);
			line(inputimg, pright_buttocks, pleft_buttocks, Scalar(0,255,255),4);
			
		}
	}
	
	return 0;
}
