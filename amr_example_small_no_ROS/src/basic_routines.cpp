#include "basic_routines.hpp"
#include "basic_structs.hpp"
#include <mutex>    //  std::mutex
#include <thread>   //  std::thread
#include <math.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <limits>
#include "yaml-cpp/yaml.h"

double AMR::determinePathLength(
    const AMR::Coordinates2D &starting_point,
    const std::vector<Coordinates2D> &part_locations,
    const AMR::Coordinates2D &delivery_point,
    const std::vector<int> &pickup_order) {
  // compute the distance between the starting point and the first part location
  double x_diff = part_locations[pickup_order[0]]._x - starting_point._x;
  double y_diff = part_locations[pickup_order[0]]._y - starting_point._y;
  double path_length = sqrt(x_diff * x_diff + y_diff * y_diff);
  // add the distances between all the parts in the specified order
  for (size_t i = 0; i < pickup_order.size() - 1; ++i) {
    x_diff = part_locations[pickup_order[i + 1]]._x -
             part_locations[pickup_order[i]]._x;
    y_diff = part_locations[pickup_order[i + 1]]._y -
             part_locations[pickup_order[i]]._y;
    path_length += sqrt(x_diff * x_diff + y_diff * y_diff);
  }
  // add the distance between the last part and the delivery point
  x_diff = delivery_point._x - part_locations.back()._x;
  y_diff = delivery_point._y - part_locations.back()._y;
  path_length += sqrt(x_diff * x_diff + y_diff * y_diff);
  return path_length;
}

void AMR::determineShortestPath(
    const AMR::Coordinates2D &starting_point,
    const std::vector<Coordinates2D> &part_locations,
    const AMR::Coordinates2D &delivery_point, std::vector<int> &pickup_order) {
  // first, prepare the output variable pickup_order
  pickup_order.resize(part_locations.size());
  
  // PLEASE ADD YOUR IMPLEMENTATION HERE
  // EXPLANATION: The vector pickup_order should be filled with integers that indicate 
  // in which order the parts in the vector part_locations should be picked up. For example, 
  // if part_locations contains 3 locations, a possible output would be {1,0,2}. This would 
  // mean that the shortest path is: 
  // starting_point, part_locations[1], part_locations[0], part_locations[2], delivery_point

  std::iota(pickup_order.begin(), pickup_order.end(), 0);

  double shortestPathLength = std::numeric_limits<double>::max();
  std::vector<int> bestOrder = pickup_order;

  // Find the one with the shortest path
  do {
      double currentPathLength = determinePathLength(starting_point, part_locations, delivery_point, pickup_order);
      if (currentPathLength < shortestPathLength) {
          shortestPathLength = currentPathLength;
          bestOrder = pickup_order;
      }
  } while (std::next_permutation(pickup_order.begin(), pickup_order.end()));

  // Set pickup_order to the best found order
  pickup_order = bestOrder;
}

void AMR::parseConfigurationFiles(
    const std::string &dir_path, std::vector<AMR::Product> &all_products,
    std::vector<AMR::ProductPart> &all_product_parts) {
  std::string configuration_file = dir_path + "/products.yaml";
  std::ifstream fin(configuration_file);
  if (fin.is_open()) {
    YAML::Node configuration_doc = YAML::Load(fin);
    long long int n_products = configuration_doc.size();
    // product ids might start at 1, so we reserve an additional position in
    // the vector to account for that.
    all_products.resize(n_products + 1);
    all_products[0]._name = "placeholder";
    for (auto product_iter = configuration_doc.begin();
         product_iter != configuration_doc.end(); ++product_iter) {
      // get the data of the current product
      long long int product_id = (*product_iter)["id"].as<long long int>();
      std::string product_name = (*product_iter)["product"].as<std::string>();
      // Add all the parts of the current product to a proper map (of part ids
      // and quantities); Update the vector of all product parts if a new part
      // is encountered.
      YAML::Node current_product_parts = (*product_iter)["parts"];
      std::map<long long int, int> parts_and_quantities;
      for (auto part_iter = current_product_parts.begin();
           part_iter != current_product_parts.end(); ++part_iter) {
        std::string part_name = (*part_iter)["part"].as<std::string>();
        // check whether the current part is already
        auto find_part_iter =
            std::find_if(all_product_parts.begin(), all_product_parts.end(),
                         [&part_name](AMR::ProductPart &part) {
                           return (part._name == part_name);
                         });
        long long part_id = 0;
        if (find_part_iter == all_product_parts.end()) {
          // new part; update the vector of all product parts
          double x_coord = (*part_iter)["cx"].as<double>();
          double y_coord = (*part_iter)["cy"].as<double>();
          all_product_parts.push_back(ProductPart(part_name, x_coord, y_coord));
          // the id of the current part is its position in the vector
          part_id = all_product_parts.size() - 1;
        } else {
          part_id = distance(all_product_parts.begin(), find_part_iter);
        }
        // add the part to the map parts_and_quantities. If the id of the part
        // is already included, update the quantity counter.
        auto part_in_map_iter = parts_and_quantities.find(part_id);
        if (part_in_map_iter == parts_and_quantities.end()) {
          parts_and_quantities.emplace(std::make_pair(part_id, 1));
        } else {
          (*part_in_map_iter).second++;
        }
      }
      all_products[product_id]._name = product_name;
      all_products[product_id]._parts = std::move(parts_and_quantities);
    }
  } else {
    std::cout << "file: " << configuration_file << " not found " << std::endl;
  }
}



void AMR::parseSingleFile(const std::string& file_path, const uint32_t order_id, AMR::Coordinates2D& delivery_point,
                          std::vector<long long int>& ordered_products, std::mutex& mutex, bool& order_found) {

    YAML::Node orders = YAML::LoadFile(file_path);

    for (const auto& order : orders) {
        if (order["order"].as<uint32_t>() == order_id) {

            std::lock_guard<std::mutex> lock(mutex);
            delivery_point._x = order["cx"].as<double>();
            delivery_point._y = order["cy"].as<double>();

            for (const auto& product : order["products"]) {
                ordered_products.push_back(product.as<long long int>());
            }
            order_found = true;
            break;
        }
    }
}

bool AMR::parseAllFilesToFindOrder(
    const std::string &dir_path, const uint32_t order_id,
    AMR::Coordinates2D &delivery_point,
    std::vector<long long int> &ordered_products) {
  // the number of files and the names of the files is hardcoded here. It
  // could be retrieved by using std::filesystem routines
  int n_files = 5;
  std::vector<std::string> file_names = {
      dir_path + "/orders_20201201.yaml", dir_path + "/orders_20201202.yaml",
      dir_path + "/orders_20201203.yaml", dir_path + "/orders_20201204.yaml",
      dir_path + "/orders_20201205.yaml"};

  // PLEASE ADD YOUR IMPLEMENTATION HERE
  // EXPLANATION: The user provides dir_path and order_id as input variables.
  // It is your task to parse the different files (in parallel) to see if any of them 
  // contains an order whose id is 'order_id'. If an order with this id is found, the 
  // corresponding delivery point coordinates (specified by cx and cy) should be parsed 
  // and written to the output variable delivery_point. Furthermore, the list of products 
  // should be parsed and the product ids should be inserted in the output vector 
  // ordered_products. 
  // If the order_id has not been found, these output values do not have to be set.
  // The routine should return a boolean indicating whether the order id was found or not.

  std::mutex mutex;
  bool order_found = false;
  std::vector<std::thread> threads;

  for (const auto& file_name : file_names) {

      threads.emplace_back(std::thread(&AMR::parseSingleFile, file_name, order_id, std::ref(delivery_point),
                                       std::ref(ordered_products), std::ref(mutex), std::ref(order_found)));

  }
    for (auto& t : threads) {
        t.join();
    }
    return order_found;
}

