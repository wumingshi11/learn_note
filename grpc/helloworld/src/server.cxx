#include <iostream>
#include <memory>
#include <string>
#include <grpcpp/grpcpp.h>
#include "fstream"
#include "service.grpc.pb.h"
#include "file_transfer.grpc.pb.h"

using namespace grpc;
using namespace example;
using namespace file_transfer;

/*
service Greeter{
  rpc SayHello(HelloRequest) returns(HelloReply) {}
}
message HelloRequest{
  string name = 1;
}
message HelloReply{
  string message = 1;
}
*/
// TODO 此示例采用同步服务模型
class FileTransferServiceImpl final : public FileTransfer::Service {
	Status UploadFile(ServerContext* context, grpc::ServerReader<FileChunk>* reader, TransferResponse* response) override {
		// std::ofstream file("uploaded_file.dat", std::ios::binary);
		/*if (!file) {
			response->set_success(false);
			response->set_message("Failed to open file for writing.");
			return Status::OK;
		}*/
		auto start = std::chrono::high_resolution_clock::now();
		FileChunk chunk;
		std::size_t n = 0;
		int c = 0;
		while (reader->Read(&chunk)) {
			n += chunk.chunk_data().size();
		//	std::cout << c++ << std::endl;
		}
		//std::cout << "read finished!" << std::endl;
		auto end = std::chrono::high_resolution_clock::now();
		auto du = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		
		response->set_success(true);
		response->set_message("File uploaded successfully. upload : ");
		std::cout << "UploadFile size : " << n << " cost : " << du << std::endl;
		return Status::OK;
	}

	
	Status DownloadFile(ServerContext* context, const FileChunk* request, grpc::ServerWriter<FileChunk>* writer) override {
		/*std::ifstream file("uploaded_file.dat", std::ios::binary);
		if (!file) {
			return Status(grpc::StatusCode::NOT_FOUND, "File not found.");
		}*/
		constexpr int BUFFER_SIZE = 1024 * 1024; // 1MB 缓冲区
		char*   buffer = new char[BUFFER_SIZE];
		auto start = std::chrono::high_resolution_clock::now();
		int64_t offset = 0;
		int64_t total_size = request->total_size();
		int tmp = 1024;
		size_t n = 0;
		while (tmp-- > 0) {
			FileChunk chunk;
			chunk.set_offset(offset);
			chunk.set_total_size(total_size);
			chunk.set_chunk_data(std::string(buffer, BUFFER_SIZE));
			
			offset = n;
			n += BUFFER_SIZE;
			writer->Write(chunk);
			
		}
		
		auto end = std::chrono::high_resolution_clock::now();
		auto du = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "form server down load : " << n << " cost : " << du << std::endl;
		delete buffer;
		buffer = nullptr;
		return Status::OK;
	}
};

class GreeterServiceImpl final : public Greeter::Service {
	Status SayHello(ServerContext* context, const HelloRequest* request,
		HelloReply* reply) override {
		std::string prefix("Hello ");
		reply->set_message(prefix + request->name());
		std::cout << "replay " + prefix + request->name() << std::endl;
		return Status::OK;
	}
};

void RunServer() {
	std::string server_address("0.0.0.0:50051");
	GreeterServiceImpl service;
	FileTransferServiceImpl fileserver;

	ServerBuilder builder;
	builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
	builder.RegisterService(&service);
	builder.RegisterService(&fileserver);
	std::unique_ptr<Server> server(builder.BuildAndStart());
	std::cout << "Server listening on " << server_address << std::endl;

	server->Wait();
}

int main() {
	RunServer();
	return 0;
}