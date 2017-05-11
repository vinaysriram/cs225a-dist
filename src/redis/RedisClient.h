#ifndef REDIS_CLIENT_H
#define REDIS_CLIENT_H

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <string>
#include <iostream>
#include <stdexcept>
#include <json/json.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>

struct HiredisServerInfo {
	std::string hostname_;
    int port_;
    timeval timeout_;
};

class RedisClient {

public:
	RedisClient () :
		context_(nullptr),
		reply_(nullptr)
	{
		// Do nothing
	}

	HiredisServerInfo server_info_;
	redisContext *context_;
    redisReply *reply_;
	//TODO: decide if needed. Currently, we throw up if server disconnects
	//bool connected_;

	/**
	 * Perform Redis command: GET key.
	 *
	 * @param key  Key to get from Redis (entry must be String type).
	 * @return     String value.
	 */
	std::string get(const std::string& key);

	/**
	 * Perform Redis command: SET key value.
	 *
	 * @param key    Key to set in Redis.
	 * @param value  Value for key.
	 */
	void set(const std::string& key, const std::string& value);

	/**
	 * Perform Redis command: MGET key1 key2...
	 *
	 * MGET gets multiple keys as an atomic operation. See:
	 * https://redis.io/commands/mget
	 *
	 * In C++11, this function can be called with brace initialization:
	 * redis_client.mset({"key1", "key2"});
	 * 
	 * @param keys  Vector of keys to get from Redis.
	 * @return      Vector of retrieved values. Optimized with RVO.
	 */
	std::vector<std::string> mget(const std::vector<std::string>& keys);

	/**
	 * Perform Redis command: MSET key1 val1 key2 val2...
	 *
	 * MSET sets multiple keys as an atomic operation. See:
	 * https://redis.io/commands/mset
	 *
	 * In C++11, this function can be called with brace initialization:
	 * redis_client.mset({{"key1", "val1"}, {"key2", "val2"}});
	 * 
	 * @param keyvals  Vector of key-value pairs to set in Redis.
	 */
	void mset(const std::vector<std::pair<std::string, std::string>>& keyvals);

	/**
 	 * Encode Eigen::MatrixXd as JSON string.
	 *
	 * For example:
	 *   [1,2,3,4]     => "[1,2,3,4]"
	 *   [[1,2],[3,4]] => "[[1,2],[3,4]]"
	 *
	 * @param matrix  Eigen::MatrixXd to encode.
	 * @return        Encoded string.
	 */
	template<typename Derived>
	static std::string encodeEigenMatrixJSON(const Eigen::MatrixBase<Derived>& matrix) {
		std::string s;
		hEigenToStringArrayJSON(matrix, s);
		return s;
	}

	/**
 	 * Decode Eigen::MatrixXd from space-delimited string.
	 *
	 * For example:
	 *   "[1,2,3,4]"     => [1,2,3,4]
	 *   "[[1,2],[3,4]]" => [[1,2],[3,4]]
	 *
	 * @param str  String to decode.
	 * @return     Decoded Eigen::Matrix.
	 */
	static Eigen::MatrixXd decodeEigenMatrixJSON(const std::string& str) {
		Eigen::MatrixXd matrix;
		hEigenFromStringArrayJSON(matrix, str);
		return matrix;
	}

	/**
 	 * Encode Eigen::MatrixXd as space-delimited string.
	 *
	 * For example:
	 *   [1,2,3,4]     => "1 2 3 4"
	 *   [[1,2],[3,4]] => "1 2; 3 4"
	 *
	 * @param matrix  Eigen::MatrixXd to encode.
	 * @return        Encoded string.
	 */
	template<typename Derived>
	static std::string encodeEigenMatrixString(const Eigen::MatrixBase<Derived>& matrix) {
		std::string s;
		hEigenToStringArrayCustom(matrix, s);
		return s;
	}

	/**
 	 * Decode Eigen::MatrixXd from space-delimited string.
	 *
	 * For example:
	 *   "1 2 3 4"  => [1,2,3,4]
	 *   "1 2; 3 4" => [[1,2],[3,4]]
	 *
	 * @param str  String to decode.
	 * @return     Decoded Eigen::Matrix.
	 */
	static Eigen::MatrixXd decodeEigenMatrixString(const std::string& str) {
		Eigen::MatrixXd matrix;
		hEigenFromStringArrayCustom(matrix, str);
		return matrix;
	}

	/**
	 * Get Eigen::MatrixXd from Redis as JSON string.
	 *
	 * @param key  Key to get from Redis.
	 * @return     Value as Eigen::MatrixXd. Optimized with RVO.
	 */
	Eigen::MatrixXd getEigenMatrixJSON(const std::string& key) {
		std::string str = get(key);
		return decodeEigenMatrixJSON(str);
		// Eigen::MatrixBase<Derived> matrix;
		// hEigenFromStringArrayJSON(matrix, str);
		// return matrix;
	}

	/**
	 * Set Eigen::MatrixXd in Redis as JSON string
	 *
	 * @param key    Key to set in Redis.
	 * @param value  Value for key.
	 */
	template<typename Derived>
	void setEigenMatrixJSON(const std::string& key, const Eigen::MatrixBase<Derived>& value) {
		std::string str = encodeEigenMatrixJSON(value);
		set(key, str);
	}

	/**
	 * Get Eigen::MatrixXd from Redis as space-delimited string.
	 *
	 * @param key  Key to get from Redis.
	 * @return     Value as Eigen::MatrixXd. Optimized with RVO.
	 */
	Eigen::MatrixXd getEigenMatrixString(const std::string& key) {
		std::string str = get(key);
		return decodeEigenMatrixString(str);
		// Eigen::MatrixBase<Derived> matrix;
		// hEigenFromStringArrayCustom(matrix, str);
		// return matrix;
	}

	/**
	 * Set Eigen::MatrixXd in Redis as space-delimited string
	 *
	 * @param key    Key to set in Redis.
	 * @param value  Value for key.
	 */
	template<typename Derived>
	void setEigenMatrixString(const std::string& key, const Eigen::MatrixBase<Derived>& value) {
		std::string str = encodeEigenMatrixString(value);
		set(key, str);
	}

public:
	// init with server info
	void serverIs(HiredisServerInfo server) {
		// delete existing connection
		if (NULL != context_ || server.hostname_.empty()) {
			redisFree(context_);
		}
		if (server.hostname_.empty()) {
			// nothing to do
			return;
		}
		// set new server info
		server_info_ = server;
		// connect to new server
		auto tmp_context = redisConnectWithTimeout(server.hostname_.c_str(), server.port_, server.timeout_);
        if (NULL == tmp_context) {
        	throw(std::runtime_error("Could not allocate redis context."));
        }
    	if (tmp_context->err) {
			std::string err = std::string("Could not connect to redis server : ") + std::string(tmp_context->errstr);
			redisFree(tmp_context);
			throw(std::runtime_error(err.c_str()));
        }
    	// set context, ping server
    	context_ = tmp_context;
	}

	// set expiry (ms) on an existing db key
	void keyExpiryIs(const std::string& key, const uint expiry_ms) {
		reply_ = (redisReply *)redisCommand(context_, "PEXPIRE %s %s", key.c_str(), std::to_string(expiry_ms).c_str());
		// NOTE: write commands dont check for write errors.
		freeReplyObject((void*)reply_);
	}

	// get command, without return string. for internal use primarily.
	bool getCommandIs(const std::string &cmd_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			throw(std::runtime_error("Server error in fetching data!"));
			//TODO: indicate what error
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// std::cout << "\nNo data on server.. Missing key?";
			return false;
		}
		return true;
	}

	// get command, with string return
	bool getCommandIs(const std::string &cmd_mssg, std::string& ret_data) {
		bool success = getCommandIs(cmd_mssg);
		if (success) {
			ret_data = reply_->str;
		}
		return success;
	}

	void setCommandIs(const std::string &cmd_mssg, const std::string &data_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "SET %s %s", cmd_mssg.c_str(), data_mssg.c_str());
		// NOTE: set commands dont check for write errors.
      	freeReplyObject((void*)reply_);
	}

	// write raw eigen vector
	template<typename Derived>
	void setEigenMatrixDerived(const std::string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
		std::string data_mssg;
		// serialize
		hEigentoStringArrayJSON(set_mat, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

    // write raw eigen vector, but in custom string format
    template<typename Derived>
    void setEigenMatrixDerivedString(const std::string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
		std::string data_mssg;
		// serialize
		hEigenToStringArrayCustom(set_mat, data_mssg);
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
    }

	// read raw eigen vector:
	template<typename Derived>
	void getEigenMatrixDerived(const std::string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hEigenFromStringArrayJSON(ret_mat, reply_->str)) {
			throw(std::runtime_error("Could not deserialize json to eigen data!"));
		}
		freeReplyObject((void*)reply_);	
	}

	// read raw eigen vector, but from a custom string rather than from json
	template<typename Derived>
	void getEigenMatrixDerivedString(const std::string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hEigenFromStringArrayCustom(ret_mat, reply_->str)) {
			throw(std::runtime_error("Could not deserialize custom string to eigen data!"));
		}
		freeReplyObject((void*)reply_);	
	}

public: // server connectivity tools
	void ping() {
		// PING server to make sure things are working..
        reply_ = (redisReply *)redisCommand(context_,"PING");
		std::cout<<"\n\nDriver Redis Task : Pinged Redis server. Reply is, "<<reply_->str<<"\n";
        freeReplyObject((void*)reply_);
	}

protected:
	template<typename Derived>
	static bool hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived> &, std::string &);

	template<typename Derived>
	static bool hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived> &, const std::string &);

	template<typename Derived>
	static bool hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived> &, std::string &);

	template<typename Derived>
	static bool hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived> &, const std::string &);

};

//Implementation must be part of header for compile time template specialization
template<typename Derived>
bool RedisClient::hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "[";
	if(row_major)
	{// [1 2 3; 4 5 6] == [ [1, 2, 3], [4, 5, 6] ]
	  for(int i=0;i<x.rows();++i){
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      if(i>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(i>0) arg_str.append(",");
	    for(int j=0;j<x.cols();++j){
	      ss<<x(i,j);
	      if(j>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	else
	{// [1 2 3; 4 5 6] == 1 4 2 5 3 6
	  for(int j=0;j<x.cols();++j){
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      if(j>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(j>0) arg_str.append(",");
	    for(int i=0;i<x.rows();++i){
	      ss<<x(i,j);
	      if(i>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	return true;
}

template<typename Derived>
bool RedisClient::hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	Json::Value jval;
    Json::Reader json_reader;
    if(!json_reader.parse(arg_str,jval))
    { return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
	  x.setIdentity(nrows,1);//Convert it into a vector.
	  for(int i=0;i<nrows;++i) x(i,0) = jval[i].asDouble();
	}
	else
	{
	  unsigned int ncols = jval[0].size();
	  x.setIdentity(nrows,ncols);
	  if(ncols < 1) return false; //Must have elements.
	  for(int i=0;i<nrows;++i){
	    if(ncols != jval[i].size()) return false;
	    for(int j=0;j<ncols;++j)
	      x(i,j) = jval[i][j].asDouble();
	  }
	}
	return true;
}

template<typename Derived>
bool RedisClient::hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "";
	if(row_major)
	{// [1 2 3; 4 5 6] == '1 2 3; 4 5 6' without the ''
	  for(int i=0;i<x.rows();++i){
	    if(i>0) { arg_str.append("; "); }
	    for(int j=0;j<x.cols();++j){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(j>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	else
	{// [1 2 3; 4 5 6] == 1 4; 2 5; 3 6
	  for(int j=0;j<x.cols();++j){
	    if(j>0) arg_str.append("; ");
	    for(int i=0;i<x.rows();++i){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(i>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	return true;
}

template<typename Derived>
bool RedisClient::hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	std::string copy_str = arg_str;
	unsigned int nrows = std::count(arg_str.begin(), arg_str.end(), ';') + 1;
	unsigned int ncols = (std::count(arg_str.begin(), arg_str.end(), ' ') + 1)/nrows;
	std::replace(copy_str.begin(), copy_str.end(), ';', ' ');

    std::stringstream ss(copy_str);

    if(nrows < 1) return false; //Must have elements.

	bool is_matrix = (nrows > 1);
	if(!is_matrix)
	{
	  x.setIdentity(ncols,1);//Convert it into a vector.
	  for(int i=0;i<ncols;++i) {
	  	std::string val;
		ss >> val;
	  	x(i,0) = std::stod(val);
	  }
	}
	else
	{
	  if(nrows < 1) return false; //Must have elements.
	  for(int i=0;i<nrows;++i){
	    for(int j=0;j<ncols;++j) {
	    	std::string val;
			ss >> val;
			x(i,j) = std::stod(val);
		}
	  }
	}
	return true;
}


#endif //REDIS_CLIENT_H
