#!/usr/bin/env ruby

require 'fileutils'

root_dir = ARGV[0]
out_dir = ARGV[1]
dirs = Dir.glob("#{root_dir}/*")

table = {}

dirs.each do |dir|
  dir_name = dir.split("/")[-1]
  serial = dir_name.split("_")[1].split(".")[0]
  table[serial] = []

  Dir.glob("#{dir}/*.pcd").each do |file|
    filename = file.split("/")[-1]
    stamp = file.gsub(/.+\/(.+)\.pcd$/, '\1')
    table[serial] |= [stamp]
  end

  table[serial].sort!
end

base_time = table.map { |k,h| h.map(&:to_f).min }.max.to_f
base_times = {}

table.each do |serial, stamps|
  stamps.each_cons(2) do |stamp1_, stamp2_|
    stamp1, stamp2 = stamp1_.to_f, stamp2_.to_f
    if (stamp1 <= base_time) && (base_time < stamp2)
      base_times[serial] = ((base_time - stamp1) < (stamp2 - base_time)) ? stamp1 : stamp2
      break
    end
  end
end

indexes = {}
table.each do |serial, stamps|
  started_at = base_times[serial]
  stamps.each.with_index do |stamp, i|
    indexes[serial] = i and break if stamp.to_f == started_at
  end
end

matched_list = []

loop do
  matched_list << Hash[indexes.map { |s, i| [s, table[s][i]] }]
  break unless indexes.keys.all? { |s| (indexes[s] += 1) < table[s].size }
end

matched_list.each.with_index do |frames, i|
  to_dir = "#{out_dir}/#{Time.at(frames.values.min.to_f).strftime('%Y%m%dT%H%M%S.%6N')}"
  FileUtils.mkdir_p(to_dir, verbose: true)
  frames.each do |serial, frame|
    from_dir = dirs.select { |d| d.include?(serial) }.first
    FileUtils.cp("#{from_dir}/#{frame}.pcd", "#{to_dir}/#{frame}_#{serial}.pcd", verbose: true)
  end
end
