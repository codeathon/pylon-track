#pragma once

#include <fstream>
#include <string>
#include <vector>

// Calibration-suite session helpers: date-labeled output folders and a
// flush-per-row CSV writer (rows survive a Ctrl-C mid-capture).

// "2026-06-10_143000" — sortable, filesystem-safe, used in folder/file names.
std::string timestamp_label();

// Creates <base>/<suite>/<timestamp>_<label>/ and returns its path.
// label may be empty (folder is then just the timestamp).
std::string make_session_dir(const std::string& base,
	const std::string& suite,
	const std::string& label);

class CsvWriter {
public:
	// Opens path and writes the header row immediately.
	CsvWriter(const std::string& path, const std::vector<std::string>& header);

	// Writes one row; flushes so partial runs keep their data.
	void write_row(const std::vector<std::string>& cells);

	bool is_open() const { return file_.is_open(); }

private:
	std::ofstream file_;
};

// Numeric cell helpers (CSV cells are strings).
std::string csv_num(double value, int precision = 3);
