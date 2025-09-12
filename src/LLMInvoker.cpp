#include <iostream>
#include <string>
#include <vector>
// #include <curl/curl.h>
// #include <nlohmann/json.hpp>

// using json = nlohmann::json;
using namespace std;

// Forward declaration of the curl and json functions, since the actual library is not included
namespace curl {
    int global_init(int flags);
    void global_cleanup();
    struct curl_slist* slist_append(struct curl_slist* list, const char* string);
    struct curl_slist* slist_free_all(struct curl_slist* list);
    void easy_setopt(void* curl, int option, ...); // Variadic function
    void* easy_init();
    CURLcode easy_perform(void* curl);
    void easy_cleanup(void* curl);
    const char* easy_strerror(CURLcode error);
}

namespace json {
    class json {
    public:
        json();
        json(const std::string& str);
        // ... other constructors
        std::string dump();
        template<typename T>
        T get();
    };

    json parse(const std::string& str);

    //class parse_error : public std::exception {}; // removed nested class, not needed for the fix
}

enum class CURLcode {
    CURLE_OK = 0,
    CURLE_FAILED_INIT,
    // ... other error codes
};

class LLMInvoker {
public:
    // Constructor
    LLMInvoker(const std::string& apiKey) : apiKey_(apiKey) {}

    // Destructor
    ~LLMInvoker() {
        if (curl_) {
            curl::easy_cleanup(curl_);
        }
        curl::global_cleanup();
    }

    // Function to send a prompt to Gemini and get a response.
    std::string sendPrompt(const std::string& prompt) {
        // 1.  Construct the API request (URL, headers, body).
        std::string apiUrl = "YOUR_GEMINI_API_ENDPOINT";
        std::string requestBody = constructRequest(prompt);

        // 2.  Make the HTTP request using libcurl.
        std::string response = makeHttpRequest(apiUrl, requestBody);

        // 3.  Parse the JSON response using nlohmann/json.
        std::string result = parseResponse(response);
        return result;
    }

private:
    std::string apiKey_;
    void* curl_ = nullptr; // cURL handle

    // Helper function to construct the JSON request body.
    std::string constructRequest(const std::string& prompt) {
        //  This is a placeholder.
        json::json requestData;
        requestData = {
            {"prompt", prompt},
            {"model", "gemini-pro"},
            // Add other parameters as required by the API
        };
        return requestData.dump();
    }

    // Helper function to make the HTTP request using libcurl.
    std::string makeHttpRequest(const std::string& url, const std::string& body) {
        std::string readBuffer;
        CURLcode res;

        if (!curl_) {
            curl::global_init(CURL_GLOBAL_DEFAULT);
            curl_ = curl::easy_init();
            if (!curl_) {
                std::cerr << "curl_easy_init() failed" << std::endl;
                return ""; // Return empty string on error
            }
        }
        struct curl_slist* headers = nullptr;
        if (curl_) {
            curl::easy_setopt(curl_, CURLOPT_URL, url.c_str());
            curl::easy_setopt(curl_, CURLOPT_POST, 1L);
            curl::easy_setopt(curl_, CURLOPT_POSTFIELDS, body.c_str());
            curl::easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
            curl::easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);
            std::string authHeader = "Authorization: Bearer " + apiKey_;
            headers = curl::slist_append(headers, authHeader.c_str());
            curl::easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);

            res = curl::easy_perform(curl_);
            if (res != CURLcode::CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl::easy_strerror(res) << std::endl;
                readBuffer = "";
            }
            curl::slist_free_all(headers);
        }
        return readBuffer;
    }

    // Callback function for libcurl to handle the response.
    static size_t writeCallback(char* contents, size_t size, size_t nmemb, std::string* output) {
        size_t totalSize = size * nmemb;
        output->append(contents, totalSize);
        return totalSize;
    }

    // Helper function to parse the JSON response.
    std::string parseResponse(const std::string& response) {
        try {
            json::json jsonResponse = json::json(response);
            std::string result = jsonResponse.get<std::string>();
            return result;
        } catch (std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            return "";
        }
    }
};
