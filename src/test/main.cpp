#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <vector>

#include "test/case/angles.hpp"
#include "test/case/circle.hpp"
#include "test/case/figure_eight.hpp"
#include "test/case/lissajous.hpp"
#include "test/case/pinocchio.hpp"
#include "test/case/pose.hpp"
#include "test/case/reach.hpp"
#include "test/case/rectangle.hpp"
#include "test/case/slerp.hpp"
#include "test/test.hpp"

/**
 * @brief Parse command line arguments.
 * 
 * @param argc The number of command line arguments.
 * @param argv The command line arguments to parse.
 * 
 * @returns A tuple containing if the parse succeeded, the parsed flags, and the
 * parsed keyword arguments.
 */
std::tuple<
    bool,
    std::unordered_set<char>,
    std::unordered_map<std::string, std::vector<std::string>>
>
parse(int argc, char **argv)
{
    std::unordered_set<char> flags;
    std::unordered_map<std::string, std::vector<std::string>> args;

    int state = 0;
    std::string key;

    for (int i = 0; i < argc; i++) {
        char *arg = argv[i];

        for (char *c = arg; c && *c;) {
            switch (state)
            {
                case 0: {
                    // Expect all arguments to begin with a dash.
                    if (*c != '-')
                        return std::make_tuple(false, flags, args);

                    c++;
                    state = 1;
                    continue;
                }

                case 1: {
                    // If there is a second dash, interpret as a key.
                    if (*c == '-') {
                        c++;
                        state = 2;
                        continue;
                    }

                    // Otherwise all subsequent characters are flags.
                    for (; *c; c++)
                        flags.emplace(*c);

                    state = 0;
                    break;
                }

                case 2: {
                    // Get the key, expect a value.
                    key = std::string(c);
                    state = 3;
                    c = nullptr;
                    break;
                }

                case 3: {
                    auto value = std::string(c);

                    // Add the value to the vector for this key.
                    auto it = args.find(key);
                    if (it == args.end())
                        args[key] = {value};
                    else
                        it->second.push_back(value);

                    state = 0;
                    c = nullptr;
                    break;
                }
            }
        }
    }

    return std::make_tuple(true, flags, args);
}

int main(int argc, char **argv)
{
    // char *program = argv[0];

    // Prints usage diagnostics for command line errors.
    auto usage = [argc, argv](std::string reason) /* [[noreturn]] */ {
        std::cerr << "usage: "<< argv[0] << " --test <string> --out <path> [--config <json>]" << std::endl;

        std::cerr << "ran: ";
        for (int i = 0; i < argc; i++)
            std::cerr << argv[i] << ' ';
        std::cerr << std::endl;

        std::cerr << "error: " << reason << std::endl;
        exit(1);
    };

    // Ignore executable name.
    argc -= 1;
    argv += 1;

    auto [success, flags, args] = parse(argc, argv);
    if (!success)
        usage("failed to parse args");

    // List all the registered tests if requested.
    if (flags.contains('l')) {
        std::cout << "available:" << std::endl;
        for (std::string &name : TestSuite::get_test_names()) {
            std::cout << "    " << name << std::endl;
        }
        return 0;
    }

    // Must supply test.
    if (!args.contains("test") || args["test"].size() != 1) 
        usage("--test must be specified");

    // Must supply output folder.
    if (!args.contains("out") || args["out"].size() != 1)
        usage("--out must be specified");

    json configuration = json::object();

    if (args.contains("config")) {
        const std::string &string = args["config"][0];
        try {
            configuration = json::parse(string);
        }
        catch (const json::exception &err) {
            using namespace std::string_literals;
            usage("failed to parse config: "s + err.what() + ", config was " + string);
        }
    }

    return (int)TestSuite::run(args["test"][0], configuration, args["out"][0]);
}
