#include<iostream>
#include<sstream>
#include<jsoncpp/json/json.h>
#include<curl/curl.h>
#include<string>
#include<exception>
 
std::string result;
 int writer(char *data, size_t size, size_t nmemb, std::string *writerData)
{
    unsigned long sizes = size * nmemb;
    if (writerData == NULL)
        return -1;
 
    writerData->append(data, sizes);
 
    return sizes;
}

std::string parseJsonResonse(std::string input)
{
   Json::Value root;
   Json::Reader reader;
   bool parsingSuccessful = reader.parse(input, root);
   if(!parsingSuccessful)
   {
       std::cout<<"!!! Failed to parse the response data"<< std::endl;
       exit(-1);
   }
   const Json::Value code = root["code"];
   const Json::Value text = root["text"];
   result = text.asString();
   //std::cout<< "response code:" << code << std::endl;
   //std::cout<< "response text:" << result << std::endl;
   return result;
}
 
/*
*   将input字腹发送到图灵服务器接受json string
*/
std::string tuling(std::string input)
{
    std::string buffer;
    std::string get_result;
    std::string strJson = "{";
    strJson += "\"key\" : \"6c426f89ed3b48cbac84460f60b7635e\","; //双引号前加／防转仪
    strJson += "\"info\" : ";
    strJson += "\"";
    strJson += input;
    strJson += "\"";
    strJson += "}";
    //std::cout<<"post json string: " << strJson << std::endl;
     try
    {
        CURL *pCurl = NULL;
        CURLcode res;
        // In windows, this will init the winsock stuff
        curl_global_init(CURL_GLOBAL_ALL);
 
        // get a curl handle
        pCurl = curl_easy_init();
        if (NULL != pCurl)
        {
            // 设置超时时间为10秒
            curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 10);
 
            // First set the URL that is about to receive our POST.
            // This URL can just as well be a
            // https:// URL if that is what should receive the data.
            curl_easy_setopt(pCurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");
            //curl_easy_setopt(pCurl, CURLOPT_URL, "http://192.168.0.2/posttest.cgi");
 
            // 设置http发送的内容类型为JSON
            curl_slist *plist = curl_slist_append(NULL,
                "Content-Type:application/json;charset=UTF-8");
            curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);
            // 设置要POST的JSON数据
            curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, strJson.c_str());
            curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, writer);
            curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);
            // Perform the request, res will get the return code
            res = curl_easy_perform(pCurl);
            // Check for errors
            if (res != CURLE_OK)
            {
                printf("curl_easy_perform() failed:%s\n", curl_easy_strerror(res));
            }
            // always cleanup
            curl_easy_cleanup(pCurl);
        }
        curl_global_cleanup();
    }
    catch (std::exception &ex)
    {
        printf("curl exception %s.\n", ex.what());
    }
    if(buffer.empty())
    {
      std::cout<< "!!! ERROR The Tuling sever response NULL" << std::endl;
    }
    else
    {
        get_result=parseJsonResonse(buffer);
    }
    //std::cout<<"get_result:"<<get_result<<std::endl;
    std::cout<<"o_o:  "<<"\033[34;4m"<<get_result<<"\033[0m"<<std::endl;

    return get_result;
}