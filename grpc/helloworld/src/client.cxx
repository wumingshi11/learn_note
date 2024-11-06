#include <iostream>
#include <memory>
#include <string>
#include <grpcpp/grpcpp.h>
#include <fstream>
#include "service.pb.h"
#include "service.grpc.pb.h"
#include "file_transfer.grpc.pb.h"

using namespace grpc;
using namespace example;
using namespace file_transfer;

/*
service Greeter {
  rpc SayHello (HelloRequest) returns (HelloReply) {}
}
message HelloRequest {
  string name = 1;
}
message HelloReply {
  string message = 1;
}
*/

class FileTransferClient {
public:
	FileTransferClient(std::shared_ptr<Channel> channel)
		: stub_(FileTransfer::NewStub(channel)) {}

	void UploadFile(const std::string& filename) {
		std::ifstream file(filename, std::ios::binary);
		if (!file) {
			std::cerr << "Failed to open file for reading." << std::endl;
			return;
		}

		TransferResponse response;
		ClientContext context;

		const int BUFFER_SIZE = 1024 * 1024; // 1MB 缓冲区
		char buffer[BUFFER_SIZE];
		int64_t offset = 0;
		int64_t total_size = GetFileSize(filename);

		std::unique_ptr<grpc::ClientWriter<FileChunk>> writer(stub_->UploadFile(&context, &response));
		while (file.read(buffer, BUFFER_SIZE)) {
			FileChunk chunk;
			chunk.set_offset(offset);
			chunk.set_total_size(total_size);
			chunk.set_chunk_data(std::string(buffer, file.gcount()));
			writer->Write(chunk);
			offset += file.gcount();
		}

		writer->WritesDone();
		Status status = writer->Finish();
		if (status.ok()) {
			std::cout << "File uploaded successfully: " << response.message() << std::endl;
		}
		else {
			std::cerr << "File upload failed: " << status.error_message() << std::endl;
		}
	}
	// 内存操作 模拟内存拷贝1GB数据
	void UploadFile2() {

		TransferResponse response;
		ClientContext context;

		const int BUFFER_SIZE = 1024 * 1024; // 1MB 缓冲区
		char* buffer = new char[BUFFER_SIZE];
		int64_t offset = 0;
		//int64_t total_size = GetFileSize(filename);
		auto start = std::chrono::high_resolution_clock::now();
		std::unique_ptr<grpc::ClientWriter<FileChunk>> writer(stub_->UploadFile(&context, &response));
		int tmp = 1024;
		while (tmp-- != 0) {
			FileChunk chunk;
			chunk.set_offset(offset);
			chunk.set_total_size(BUFFER_SIZE*1024);
			chunk.set_chunk_data(std::string(buffer, BUFFER_SIZE));
			offset += BUFFER_SIZE;
			writer->Write(chunk);
		}
		std::cout << "write finished " << std::endl;
		
		writer->WritesDone();
		Status status = writer->Finish();
		auto end = std::chrono::high_resolution_clock::now();
		
		auto du = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		if (status.ok()) {
			std::cout << "File uploaded successfully: " << response.message() << "cost : "  << du << std::endl;
		}
		else {
			std::cerr << "File upload failed: " << status.error_message() << std::endl;
		}
		delete buffer;
		buffer = nullptr;
	}

	void DownloadFile(const std::string& filename) {
		FileChunk request;
		request.set_total_size(GetFileSize(filename));

		ClientContext context;
		std::unique_ptr<grpc::ClientReader<FileChunk>> reader(stub_->DownloadFile(&context, request));

		std::ofstream file("downloaded_file.dat", std::ios::binary);
		if (!file) {
			std::cerr << "Failed to open file for writing." << std::endl;
			return;
		}

		FileChunk chunk;
		while (reader->Read(&chunk)) {
			file.write(reinterpret_cast<const char*>(chunk.chunk_data().data()), chunk.chunk_data().size());
		}

		file.close();
		Status status = reader->Finish();
		if (status.ok()) {
			std::cout << "File downloaded successfully." << std::endl;
		}
		else {
			std::cerr << "File download failed: " << status.error_message() << std::endl;
		}
	}

	void DownloadFile2() {
		FileChunk request;
		// request.set_total_size(GetFileSize(filename));
		auto start = std::chrono::high_resolution_clock::now();
		ClientContext context;
		std::unique_ptr<grpc::ClientReader<FileChunk>> reader(stub_->DownloadFile(&context, request));

		FileChunk chunk;
		std::size_t n = 0;
		while (reader->Read(&chunk)) {
			n += chunk.chunk_data().size();
		}

		Status status = reader->Finish();
		auto end = std::chrono::high_resolution_clock::now();
		auto du = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		if (status.ok()) {
			std::cout << "File downloaded successfully." << std::endl;
			std::cout << "download : " << n << "cost : " << du << std::endl;
		}
		else {
			std::cerr << "File download failed: " << status.error_message() << std::endl;
		}
	}

private:
	std::unique_ptr<FileTransfer::Stub> stub_;

	int64_t GetFileSize(const std::string& filename) {
		std::ifstream file(filename, std::ios::ate | std::ios::binary);
		if (file) {
			return file.tellg();
		}
		return 0;
	}
};

class GreeterClient {
public:
	GreeterClient(std::shared_ptr<Channel> channel)
		: stub_(Greeter::NewStub(channel)) {}

	std::string SayHello(const std::string& user) {
		HelloRequest request;
		request.set_name(user);

		HelloReply reply;
		ClientContext context;

		Status status = stub_->SayHello(&context, request, &reply);

		if (status.ok()) {
			return reply.message();
		}
		else {
			std::cout << status.error_code() << ": " << status.error_message()
				<< std::endl;
			return "RPC failed";
		}
	}

private:
	// 客户端，只管发消息，不需要继承重写
	std::unique_ptr<Greeter::Stub> stub_;
};

int main() {
	string ip = "192.168.1.3:50051";
	GreeterClient greeter(grpc::CreateChannel(
		ip, grpc::InsecureChannelCredentials()));
	std::string user("world");
	std::string reply = greeter.SayHello(user);
	std::cout << "Greeter received: " << reply << std::endl;

	FileTransferClient client(grpc::CreateChannel(ip, grpc::InsecureChannelCredentials()));
	// 上传文件
	client.UploadFile2();
	// 下载文件
	client.DownloadFile2();
	//
	client.UploadFile2();
	client.DownloadFile2();
	
	return 0;
}