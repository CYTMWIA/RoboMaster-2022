#include <opencv2/opencv.hpp>

#include "oatpp/core/macro/codegen.hpp"
#include "oatpp/core/macro/component.hpp"
#include "oatpp/network/Server.hpp"
#include "oatpp/network/tcp/server/ConnectionProvider.hpp"
#include "oatpp/web/server/api/ApiController.hpp"
#include "oatpp/web/server/HttpConnectionHandler.hpp"

#include "threading.hpp"
#include "capture.hpp"

#include "web.hpp"

namespace rmcv
{
#include OATPP_CODEGEN_BEGIN(ApiController)
    class RmcvApiController : public oatpp::web::server::api::ApiController {
    public:
        RmcvApiController()
            :oatpp::web::server::api::ApiController(std::shared_ptr<ObjectMapper>()) 
        {}

        ENDPOINT("GET", "/", root) {
            return createResponse(Status::CODE_200, "Hello RMCV!");
        }

        ENDPOINT("GET", "/api/image/{name}", api_image, PATH(String, name)) {
            if (name=="capture")
            {
                std::vector<uchar> buf;
                cv::imencode(".jpeg", GlobalVariable<cv::Mat>::get("capture_image", true), buf);
                // std::string_view s(buf.data(), std::size(buf));
                auto rsp = createResponse(Status::CODE_200, oatpp::String((char*)buf.data(), buf.size()));
                rsp->putHeader("Content-Type", "image/jpeg");
                return rsp;
            }

            return createResponse(Status::CODE_404, "Image Not Found");
        }
    };
#include OATPP_CODEGEN_END(ApiController)

    void run(oatpp::network::Address address)
    {
        /* Create Router for HTTP requests routing */
        auto router = oatpp::web::server::HttpRouter::createShared();

        router->addController(std::make_shared<RmcvApiController>());
    
        /* Create HTTP connection handler with router */
        auto connectionHandler = oatpp::web::server::HttpConnectionHandler::createShared(router);

        /* Create TCP connection provider */
        auto connectionProvider = oatpp::network::tcp::server::ConnectionProvider::createShared(address);

        /* Create server which takes provided TCP connections and passes them to HTTP connection handler */
        oatpp::network::Server server(connectionProvider, connectionHandler);

        /* Print info about server port */
        OATPP_LOGI("RMCV Web", "Server running on port %s", connectionProvider->getProperty("port").getData());

        /* Run server */
        server.run();
    }

    void start_web(const Config::Web& cfg)
    {
        /* Init oatpp Environment */
        oatpp::base::Environment::init();

        /* Run App */
        run({cfg.ip, cfg.port, oatpp::network::Address::IP_4});

        /* Destroy oatpp Environment */
        oatpp::base::Environment::destroy();
    }
}