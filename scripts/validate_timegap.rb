#!/usr/bin/env ruby

root_dir = ARGV[0]
gaps = []

Dir.glob("#{root_dir}/*").each do |dir|
  stamps = Dir.glob("#{dir}/*.pcd").map { |file| file.gsub(/.+\/([\d\.]+)_\d+\.pcd$/, '\1').to_f }
  gaps << stamps.max - stamps.min
end

ave = gaps.inject(:+) / gaps.size
var = (gaps.inject { |s, n| s + n ** 2 } / gaps.size) - ave ** 2

puts "Interframe gaps"
puts "  max gap:\t#{gaps.max * 1000} ms"
puts "  min gap:\t#{gaps.min * 1000} ms"
puts "  average:\t#{ave * 1000} ms"
puts "  variance:\t#{var}"

