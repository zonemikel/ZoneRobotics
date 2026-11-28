mkdir -p resized
for img in *.png *.JPG *.jpeg *.webp; do
  [ -e "$img" ] || continue
  convert "$img" -define jpeg:extent=500kb "resized/${img%.*}.jpg"
done
