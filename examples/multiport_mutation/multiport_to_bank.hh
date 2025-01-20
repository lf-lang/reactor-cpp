//
// Created by tanneberger on 1/13/25.
//

#ifndef MULTIPORT_TO_BANK_HH
#define MULTIPORT_TO_BANK_HH

#include <reactor-cpp/mutations.hh>
#include <reactor-cpp/multiport.hh>
#include <reactor-cpp/port.hh>
#include <reactor-cpp/mutations/multiport.hh>
#include <reactor-cpp/mutations/bank.hh>
#include <reactor-cpp/mutations/connection.hh>
#include <reactor-cpp/reactor.hh>

#include "../../lib/mutations/bank.cc"
#include "../../lib/mutations/connection.cc"
#include "../../lib/mutations/multiport.cc"

#include <functional>

namespace reactor {

  template<class PortType, class ReactorType>
  class ResizeMultiportToBank : public Mutation {
     ModifableMultiport<Output<PortType>>* multiport_;
     std::vector<std::unique_ptr<ReactorType>>* bank_;
     std::function<Input<PortType>*(const std::unique_ptr<ReactorType>&)> get_input_port_;
     std::function<std::unique_ptr<ReactorType>(Environment* env, std::size_t index)> create_lambda_;
     std::size_t new_size_ = 0;
     public:
       ResizeMultiportToBank(ModifableMultiport<Output<PortType>>* multiport,
                             std::vector<std::unique_ptr<ReactorType>>* bank,
                             std::function<Input<PortType>*(const std::unique_ptr<ReactorType>&)> get_input_port,
                             std::function<std::unique_ptr<ReactorType>(Environment* env, std::size_t index)> create_lambda,
                             std::size_t new_size) :
         multiport_(multiport), bank_(bank), get_input_port_(get_input_port), create_lambda_(create_lambda), new_size_(new_size) {}

       ~ResizeMultiportToBank() = default;
       auto run() -> MutationResult {
         if (multiport_->size() != bank_->size()) {
            return NotMatchingBankSize;
         }
         auto old_size = multiport_->size();

         if (new_size_ > old_size) {
            // TODO: this is an assumption
            auto change_multiport_size =
              std::make_shared<MutationChangeOutputMultiportSize<unsigned>>(multiport_, new_size_);

            change_multiport_size->run();

            auto change_bank_size = std::make_shared<MutationChangeBankSize<std::unique_ptr<ReactorType>>>(
              bank_, (*bank_)[0]->environment(), new_size_, create_lambda_);

            change_bank_size->run();

            for (auto i = old_size; i < new_size_; i++) {
               auto add_conn = std::make_shared<MutationAddConnection<Output<PortType>, Input<PortType>>>(
                  &(*multiport_)[i], get_input_port_((*bank_)[i]), (*bank_)[0]->environment(), true);

               add_conn->run();
            }
         } else if (new_size_ < old_size) {
           for (auto i = old_size - 1; i >= new_size_; i--) {
               auto add_conn = std::make_shared<MutationAddConnection<Output<PortType>, Input<PortType>>>(
                  &(*multiport_)[i], get_input_port_((*bank_)[i]), (*bank_)[0]->environment(), false);

               add_conn->run();
            }

            auto change_multiport_size =
              std::make_shared<MutationChangeOutputMultiportSize<unsigned>>(multiport_, new_size_);

            change_multiport_size->run();

            auto change_bank_size = std::make_shared<MutationChangeBankSize<std::unique_ptr<ReactorType>>>(
                 bank_, (*bank_)[0]->environment(), new_size_, create_lambda_);

            change_bank_size->run();
         }


         return Success;
       }

       auto rollback() -> MutationResult {
        return Success;
       }
  };
}



#endif //MULTIPORT_TO_BANK_HH
