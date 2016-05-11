#!/usr/bin/env ruby

require 'date'
require 'fileutils'

root_dir = ARGV[0]

Dir.glob("#{root_dir}/*").each do |dir|
  dir_name = dir.split("/")[-1]
  serial = dir_name.split("_")[1].split(".")[0]
  started_at = DateTime.parse(dir_name.split("_")[0]).to_time

  Dir.glob("#{dir}/*.pcd").each do |file|
    filename = file.split("/")[-1]
    rel_stamp = file.gsub(/.+\/(.+)\.pcd$/, '\1')
    abs_stamp = (started_at + (rel_stamp.to_f / 1000 / 1000)).strftime("%s.%N")
    new_file = file.gsub("#{rel_stamp}.pcd", "#{abs_stamp}.pcd")
    FileUtils.mv(file, new_file, verbose: true)
  end
end

