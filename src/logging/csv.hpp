#pragma once

#include <vector>

#include "logging/file.hpp"

namespace logger {

/**
 * @brief A concept that checks if a data structure is iterable.
 */
template<typename T>
concept Iterable = requires(T iterable)
{
    std::begin(iterable) != std::end(iterable);
    ++std::declval<decltype(std::begin(iterable))&>();
    *std::begin(iterable);
};

/**
 * @brief A CSV logging class.
 */
class CSV
{
public:

    /**
     * @brief The first line in the CSV file naming to columns.
     */
    using Header = std::vector<std::string>;

    /**
     * @brief Configuration of the CSV file.
     */
    struct Configuration {

        /// The file to log to.
        std::filesystem::path path;

        /// The csv column headers.
        Header header;
    };

    /**
     * @brief Convenience function to make a CSV header.
     * 
     * @param args The elements of the header.
     * @returns A vector of strings as the header 
     */
    template<typename... Args>
    static inline Header make_header(Args... args)
    {
        Header header;
        (push_header_element(header, std::forward<Args>(args)), ...);
        return header;
    }

    /**
     * @brief Create a new CSV logger.
     * 
     * @param configuration 
     * @return std::unique_ptr<CSV> 
     */
    static inline std::unique_ptr<CSV> create(
        const Configuration &configuration
    ) {
        using namespace std::string_literals;

        auto file = File::create(configuration.path);
        if (!file) {
            std::cerr << "failed to create csv log file" << std::endl;
            return nullptr;
        }

        // Output the csv header.
        if (!configuration.header.empty()) {
            *file << configuration.header[0];
            for (std::size_t i = 1; i < configuration.header.size(); i++)
                *file << ", " << configuration.header[i];
            *file << '\n';
        }

        auto csv = std::unique_ptr<CSV>(new CSV());
        csv->m_file = std::move(file);

        return csv;
    }

    /**
     * @brief Write a row the CSV file.
     * 
     * This function generically handles writing variables to a CSV file. If the
     * variable is iterable, then the csv file will iterate over each element
     * and log it separately.
     * 
     * @param arg The first argument passed to write.
     * @param args Optional other arguments to write to the file.
     */
    template<typename Arg, typename... Args>
    void write(Arg &&arg, Args&&... args)
    {
        using namespace std::string_literals;

        // Log the first value.
        write_value(std::forward<Arg&&>(arg));

        // Log comma separated values.
        ((*m_file << ", ", write_value(std::forward<Args&&>(args))), ...);

        // End with newline. End of newline at end of file.
        *m_file << '\n';
    }

    /**
     * @brief Flush to disk.
     */
    inline void flush() {
        m_file->get_stream().flush();
    }

private:

    CSV() = default;

    /**
     * @brief Push a string to a header.
     */
    static inline void push_header_element(Header &header, std::string string) {
        header.push_back(string);
    }

    /**
     * @brief Push an iterable of strings to a header.
     */
    template<typename T>
    static inline void push_header_element(Header &header, const T &iterable)
        requires Iterable<T>
    {
        for (auto &element : iterable)
            header.push_back(element);
    }

    /**
     * @brief Write an iterable to the file, comma separated.
     * 
     * @tparam T The type of iterable to write.
     * @param iterable The iterable to write.
     */
    template<typename T>
    void write_value(const T &iterable) requires Iterable<T>
    {
        auto it = std::begin(iterable);
        if (it == std::end(iterable))
            return;

        *m_file << *it;

        // Comma separation after the first element.
        for (++it; it != std::end(iterable); ++it) {
            *m_file << ", " << *it;
        }
    }

    /**
     * @brief Write a value to the CSV file.
     * 
     * @tparam T The type of the value.
     * @param value The value to write.
     */
    template<typename T>
    void write_value(const T &value) {
        *m_file << value;
    }

    /// The file to write to.
    std::unique_ptr<File> m_file;
};

} // namespace logger
