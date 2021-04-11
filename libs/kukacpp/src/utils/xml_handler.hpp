#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include <string.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "models.hpp"
#include <vector>
#include <sstream> // stringstream

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 4)
{
    std::ostringstream out;
    out << std::fixed;
    out.precision(n);
    out << a_value;
    return out.str();
}

namespace xml_handler
{

    kuka::Data get_data_from_xml_send(char *xml_send, int client_msg_len)
    {
        kuka::Data data = kuka::Data();

        rapidxml::xml_document<> doc;
        rapidxml::xml_node<> *root_node;

        doc.parse<0>(xml_send);

        root_node = doc.first_node("Rob");

        if (client_msg_len > 256)
        {
            rapidxml::xml_node<> *rist_node = root_node->first_node("RIst");
            kuka::KukaPose r_ist;
            r_ist.set_x(atof(rist_node->first_attribute("X")->value()));
            r_ist.set_y(atof(rist_node->first_attribute("Y")->value()));
            r_ist.set_z(atof(rist_node->first_attribute("Z")->value()));
            r_ist.set_a(atof(rist_node->first_attribute("A")->value()));
            r_ist.set_b(atof(rist_node->first_attribute("B")->value()));
            r_ist.set_c(atof(rist_node->first_attribute("C")->value()));
            data.set_r_ist(r_ist);
            rapidxml::xml_node<> *rsol_node = root_node->first_node("RSol");
            kuka::KukaPose r_sol;
            r_sol.set_x(atof(rsol_node->first_attribute("X")->value()));
            r_sol.set_y(atof(rsol_node->first_attribute("Y")->value()));
            r_sol.set_z(atof(rsol_node->first_attribute("Z")->value()));
            r_sol.set_a(atof(rsol_node->first_attribute("A")->value()));
            r_sol.set_b(atof(rsol_node->first_attribute("B")->value()));
            r_sol.set_c(atof(rsol_node->first_attribute("C")->value()));
            data.set_r_sol(r_sol);
            rapidxml::xml_node<> *aipos_node = root_node->first_node("AIPos");
            kuka::Axes ai_pos;
            ai_pos.set_a1(atof(aipos_node->first_attribute("A1")->value()));
            ai_pos.set_a2(atof(aipos_node->first_attribute("A2")->value()));
            ai_pos.set_a3(atof(aipos_node->first_attribute("A3")->value()));
            ai_pos.set_a4(atof(aipos_node->first_attribute("A4")->value()));
            ai_pos.set_a5(atof(aipos_node->first_attribute("A5")->value()));
            ai_pos.set_a6(atof(aipos_node->first_attribute("A6")->value()));
            data.set_ai_pos(ai_pos);
            rapidxml::xml_node<> *aspos_node = root_node->first_node("ASPos");
            kuka::Axes as_pos;
            as_pos.set_a1(atof(aspos_node->first_attribute("A1")->value()));
            as_pos.set_a2(atof(aspos_node->first_attribute("A2")->value()));
            as_pos.set_a3(atof(aspos_node->first_attribute("A3")->value()));
            as_pos.set_a4(atof(aspos_node->first_attribute("A4")->value()));
            as_pos.set_a5(atof(aspos_node->first_attribute("A5")->value()));
            as_pos.set_a6(atof(aspos_node->first_attribute("A6")->value()));
            data.set_as_pos(as_pos);
            rapidxml::xml_node<> *eipos_node = root_node->first_node("EIPos");
            kuka::Axes ei_pos;
            ei_pos.set_a1(atof(eipos_node->first_attribute("E1")->value()));
            ei_pos.set_a2(atof(eipos_node->first_attribute("E2")->value()));
            ei_pos.set_a3(atof(eipos_node->first_attribute("E3")->value()));
            ei_pos.set_a4(atof(eipos_node->first_attribute("E4")->value()));
            ei_pos.set_a5(atof(eipos_node->first_attribute("E5")->value()));
            ei_pos.set_a6(atof(eipos_node->first_attribute("E6")->value()));
            data.set_ei_pos(ei_pos);
            rapidxml::xml_node<> *espos_node = root_node->first_node("ESPos");
            kuka::Axes es_pos;
            es_pos.set_a1(atof(espos_node->first_attribute("E1")->value()));
            es_pos.set_a2(atof(espos_node->first_attribute("E2")->value()));
            es_pos.set_a3(atof(espos_node->first_attribute("E3")->value()));
            es_pos.set_a4(atof(espos_node->first_attribute("E4")->value()));
            es_pos.set_a5(atof(espos_node->first_attribute("E5")->value()));
            es_pos.set_a6(atof(espos_node->first_attribute("E6")->value()));
            data.set_es_pos(es_pos);
            rapidxml::xml_node<> *macur_node = root_node->first_node("MACur");
            kuka::Axes ma_cur;
            ma_cur.set_a1(atof(macur_node->first_attribute("A1")->value()));
            ma_cur.set_a2(atof(macur_node->first_attribute("A2")->value()));
            ma_cur.set_a3(atof(macur_node->first_attribute("A3")->value()));
            ma_cur.set_a4(atof(macur_node->first_attribute("A4")->value()));
            ma_cur.set_a5(atof(macur_node->first_attribute("A5")->value()));
            ma_cur.set_a6(atof(macur_node->first_attribute("A6")->value()));
            data.set_ma_cur(ma_cur);
            rapidxml::xml_node<> *mecur_node = root_node->first_node("MECur");
            kuka::Axes me_cur;
            me_cur.set_a1(atof(mecur_node->first_attribute("E1")->value()));
            me_cur.set_a2(atof(mecur_node->first_attribute("E2")->value()));
            me_cur.set_a3(atof(mecur_node->first_attribute("E3")->value()));
            me_cur.set_a4(atof(mecur_node->first_attribute("E4")->value()));
            me_cur.set_a5(atof(mecur_node->first_attribute("E5")->value()));
            me_cur.set_a6(atof(mecur_node->first_attribute("E6")->value()));
            data.set_me_cur(me_cur);
            rapidxml::xml_node<> *delay_node = root_node->first_node("Delay");
            data.set_delay(atoi(delay_node->first_attribute("D")->value()));
            rapidxml::xml_node<> *digout_node = root_node->first_node("Digout");
            kuka::DigitalOutputs dig_out;
            dig_out.set_o1(atoi(digout_node->first_attribute("o1")->value()));
            dig_out.set_o2(atoi(digout_node->first_attribute("o2")->value()));
            dig_out.set_o3(atoi(digout_node->first_attribute("o3")->value()));
            data.set_digital_outputs(dig_out);
            rapidxml::xml_node<> *ftc_node = root_node->first_node("FTC");
            kuka::KukaPose ftc;
            ftc.set_x(atof(ftc_node->first_attribute("Fx")->value()));
            ftc.set_y(atof(ftc_node->first_attribute("Fy")->value()));
            ftc.set_z(atof(ftc_node->first_attribute("Fz")->value()));
            ftc.set_a(atof(ftc_node->first_attribute("Mx")->value()));
            ftc.set_b(atof(ftc_node->first_attribute("My")->value()));
            ftc.set_c(atof(ftc_node->first_attribute("Mz")->value()));
            data.set_ftc(ftc);
            rapidxml::xml_node<> *ipoc_node = root_node->first_node("IPOC");
            data.set_ipoc(atoi(ipoc_node->value()));
        }
        else
        {
            rapidxml::xml_node<> *ipoc_node = root_node->first_node("IPOC");
            data.set_ipoc(atoi(ipoc_node->value()));
        }

        return data;
    }

    std::string generate_xml_receive(kuka::KukaPose move = kuka::KukaPose(), kuka::Axes rotate = kuka::Axes(),
                                    kuka::DigitalOutputs dig_out = kuka::DigitalOutputs(), unsigned int ipoc = 145)
    {
        // Write xml file =================================
        rapidxml::xml_document<> doc;

        rapidxml::xml_node<> *root_node = doc.allocate_node(rapidxml::node_element, "Sen");
        root_node->append_attribute(doc.allocate_attribute("Type", "ImFree"));
        doc.append_node(root_node);

        rapidxml::xml_node<> *estr_node = doc.allocate_node(rapidxml::node_element, "EStr");
        estr_node->value("Laboratorio de Manipulacao Robotica - EESC USP");
        root_node->append_node(estr_node);

        rapidxml::xml_node<> *rkorr_node = doc.allocate_node(rapidxml::node_element, "RKorr");
        rkorr_node->append_attribute(doc.allocate_attribute("A", to_string_with_precision<float>(move.get_a()).c_str()));
        rkorr_node->append_attribute(doc.allocate_attribute("B", to_string_with_precision<float>(move.get_b()).c_str()));
        rkorr_node->append_attribute(doc.allocate_attribute("C", to_string_with_precision<float>(move.get_c()).c_str()));
        rkorr_node->append_attribute(doc.allocate_attribute("X", to_string_with_precision<float>(move.get_x()).c_str()));
        rkorr_node->append_attribute(doc.allocate_attribute("Y", to_string_with_precision<float>(move.get_y()).c_str()));
        rkorr_node->append_attribute(doc.allocate_attribute("Z", to_string_with_precision<float>(move.get_z()).c_str()));
        root_node->append_node(rkorr_node);

        rapidxml::xml_node<> *akorr_node = doc.allocate_node(rapidxml::node_element, "AKorr");
        akorr_node->append_attribute(doc.allocate_attribute("A1", to_string_with_precision<float>(rotate.get_a1()).c_str()));
        akorr_node->append_attribute(doc.allocate_attribute("A2", to_string_with_precision<float>(rotate.get_a2()).c_str()));
        akorr_node->append_attribute(doc.allocate_attribute("A3", to_string_with_precision<float>(rotate.get_a3()).c_str()));
        akorr_node->append_attribute(doc.allocate_attribute("A4", to_string_with_precision<float>(rotate.get_a4()).c_str()));
        akorr_node->append_attribute(doc.allocate_attribute("A5", to_string_with_precision<float>(rotate.get_a5()).c_str()));
        akorr_node->append_attribute(doc.allocate_attribute("A6", to_string_with_precision<float>(rotate.get_a6()).c_str()));
        root_node->append_node(akorr_node);

        rapidxml::xml_node<> *ekorr_node = doc.allocate_node(rapidxml::node_element, "EKorr");
        ekorr_node->append_attribute(doc.allocate_attribute("E1", "0.0000"));
        ekorr_node->append_attribute(doc.allocate_attribute("E2", "0.0000"));
        ekorr_node->append_attribute(doc.allocate_attribute("E3", "0.0000"));
        ekorr_node->append_attribute(doc.allocate_attribute("E4", "0.0000"));
        ekorr_node->append_attribute(doc.allocate_attribute("E5", "0.0000"));
        ekorr_node->append_attribute(doc.allocate_attribute("E6", "0.0000"));
        root_node->append_node(ekorr_node);

        rapidxml::xml_node<> *tech_node = doc.allocate_node(rapidxml::node_element, "Tech");
        tech_node->append_attribute(doc.allocate_attribute("T21", "1.09"));
        tech_node->append_attribute(doc.allocate_attribute("T210", "10.00"));
        tech_node->append_attribute(doc.allocate_attribute("T22", "2.08"));
        tech_node->append_attribute(doc.allocate_attribute("T23", "3.07"));
        tech_node->append_attribute(doc.allocate_attribute("T24", "4.06"));
        tech_node->append_attribute(doc.allocate_attribute("T25", "5.05"));
        tech_node->append_attribute(doc.allocate_attribute("T26", "6.04"));
        tech_node->append_attribute(doc.allocate_attribute("T27", "7.03"));
        tech_node->append_attribute(doc.allocate_attribute("T28", "8.02"));
        tech_node->append_attribute(doc.allocate_attribute("T29", "9.01"));
        root_node->append_node(tech_node);

        rapidxml::xml_node<> *digout_node = doc.allocate_node(rapidxml::node_element, "DigOut");
        digout_node->append_attribute(doc.allocate_attribute("D1", to_string_with_precision<bool>(dig_out.get_o1(), 0).c_str()));
        digout_node->append_attribute(doc.allocate_attribute("D2", to_string_with_precision<bool>(dig_out.get_o2(), 0).c_str()));
        digout_node->append_attribute(doc.allocate_attribute("D3", to_string_with_precision<bool>(dig_out.get_o3(), 0).c_str()));
        root_node->append_node(digout_node);

        rapidxml::xml_node<> *ipoc_node = doc.allocate_node(rapidxml::node_element, "IPOC");
        ipoc_node->value(std::to_string(ipoc).c_str());
        root_node->append_node(ipoc_node);

        //std::cout << "XML Receive: " << std::endl;
        //std::cout << doc;
        //std::cout << "=======================================================" << std::endl;

        // Convert doc to string if needed
        std::string xml_as_string;
        rapidxml::print(std::back_inserter(xml_as_string), doc);

        return xml_as_string;

        /* Save to file
        std::ofstream file_stored("file_stored.xml");
        file_stored << doc;
        file_stored.close();
        doc.clear();*/
    }

} // namespace xml_handler
