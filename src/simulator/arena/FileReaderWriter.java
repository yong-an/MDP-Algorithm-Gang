package simulator.arena;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

/* This Java file handles the printing of the map descriptor into a text file.*/

public class FileReaderWriter {
	
	private static final String DEFAULT_ENCODING = "UTF-8";
	private Path _file;
	private OutputStreamWriter _writer;
	private BufferedReader _reader;
	
	//Constructor will generate a new text file for the map descriptor.
	public FileReaderWriter(Path file) throws IOException {
		_file = file;
		if (!Files.exists(_file)) {
			Files.createDirectories(_file.getParent());
			Files.createFile(_file);
		}
	}
	
	//Function to handle the writing onto a text file.
	public void write(String str) throws IOException {
		OutputStream out = Files.newOutputStream(_file, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
		_writer = new OutputStreamWriter(out, DEFAULT_ENCODING);
		_writer.write(str);
		_writer.close();
	}
	
	//Function to read the map descriptor text file.
	public String read() throws IOException {
		InputStream in = Files.newInputStream(_file);
		_reader = new BufferedReader(new InputStreamReader(in, DEFAULT_ENCODING));
		String text = "";
		String line;
		while ((line = _reader.readLine()) != null) {
			text += line;
		}
		_reader.close();
		return text;
	}
	
}
